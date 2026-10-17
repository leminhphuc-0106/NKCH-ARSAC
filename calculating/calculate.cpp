#include "Calculate_Lib.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <devguid.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <regex>
#include <setupapi.h>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <windows.h>

using namespace std;
using namespace chrono;

mutex programMutex;
mutex serialMutex;
mutex consoleMutex;
// ===== TELEMETRY BUFFER =====
// Protects all trajectory-specific telemetry state below.
mutex telemetryMutex;
steady_clock::time_point trajectoryStart;
steady_clock::time_point trajectoryTelemetryStart;
uint64_t telemetryPacketCount = 0;
int16_t lastDesiredCount1 = 0;
int16_t lastDesiredCount2 = 0;
// Dynamics feedforward torque (N*m) for the reference sample most recently
// transmitted; attached to each recorded telemetry sample (same 10 ms grid).
float lastTau1 = 0.0f;
float lastTau2 = 0.0f;
const std::string TELEMETRY_FILE_PATH =
    "E:\\CODE\\Arduino\\codecode\\telemetry.csv";

#define DEBUG_TELEMETRY 0

Command cmd;

// Emergency stop and homing
#define HOME1 15
#define HOME2 16
#define STOP 17

#define pwm_num1 11880
#define pwm_num2 1980

DATAA indata;

MOVE_MODE Mode = NORMAL;
bool loadingProgram = false;
bool runProgram = false;

HANDLE serialHandle = INVALID_HANDLE_VALUE;
double currentQ1 = 0.0;
double currentQ2 = PI;
bool trajectoryActive = false;
double trajectoryDuration = 0.0;

uint16_t totalCommand = 0, loadedCommand = 0, executedCommand = 0;

// Debug instrumentation counters
uint64_t controlCycle = 0;
uint64_t txPacketCount = 0;
uint64_t trajectoryInterpolationCycles = 0;

point Point = {0}, startPoint = {0}, targetPoint = {0};
joint theta = {0}, startTheta = {0}, targetTheta = {0};
stateTheta ThetaState = {0};
circle Circle = {0};

// Precomputed joint samples for LINE / CIRCLE trajectories.  Filled by the
// existing DATAA planners (RRplanar_in + jointToCounts) when a command is
// dequeued, then streamed through the unchanged motor-control path.
const int MAX_PLAN_SAMPLES = 4096;
vector<joint> activeJointTrajectory;
bool activeUsesJointTrajectory = false;

boolean checkData = false, isDone1 = false, isDone2 = false, isHoming1 = false,
        isHoming2 = false, isMoving = false, isStop = false,
        isStartPoint1 = false, isStartPoint2 = false;

char bug[2];

uint8_t buffer[20];
uint8_t length = 0;
char line[50];
int32_t pulseData1 = 0, pulseData2 = 0;

uint8_t cmdBuffer[20];
uint8_t cmdLength = 0;
boolean hasCmd = false;

enum class TelemetryParserState {
  WaitHeader1,
  WaitHeader2,
  ReadData,
  ReadChecksum
};

struct EncoderTelemetry {
  double pcTime;
  int16_t desiredCount1;
  int16_t desiredCount2;
  int32_t count1;
  int32_t count2;
  double q1Actual;
  double q2Actual;
  float tau1; // dynamics feedforward for the matching reference sample (N*m)
  float tau2;
};

// ===== TELEMETRY BUFFER =====
vector<EncoderTelemetry> telemetryBuffer;
bool telemetryRecording = false;
uint64_t trajectoryID = 0;
mutex trajectoryFileMutex;
int16_t trajectoryStartCount1 = 0;
int16_t trajectoryStartCount2 = 0;
int16_t latestActualCount1 = 0;
int16_t latestActualCount2 = 0;
bool haveActualTelemetry = false;

CMD_TYPE activeTrajectoryType = NONE;

bool waitForActualTelemetry() {
  for (int attempt = 0; attempt < 250; ++attempt) {
    {
      lock_guard<mutex> lock(telemetryMutex);
      if (haveActualTelemetry)
        return true;
    }
    this_thread::sleep_for(chrono::milliseconds(1));
  }
  return false;
}

filesystem::path trajectoryDirectoryFor(CMD_TYPE type) {
  const filesystem::path root("E:\\CODE\\Arduino\\codecode\\Data");
  switch (type) {
  case POINT_CMD:
    return root / "Point";
  case LINE:
    return root / "Line";
  case CIRCLE:
    return root / "Circle";
  default:
    return root / "Unknown";
  }
}

class TelemetryParser {
public:
  bool push(uint8_t byteIn, int16_t &count1, int16_t &count2) {
    switch (state_) {
    case TelemetryParserState::WaitHeader1:
      if (byteIn == 0x55)
        state_ = TelemetryParserState::WaitHeader2;
      break;
    case TelemetryParserState::WaitHeader2:
      if (byteIn == 0xAA) {
        dataIndex_ = 0;
        state_ = TelemetryParserState::ReadData;
      } else {
        state_ = (byteIn == 0x55) ? TelemetryParserState::WaitHeader2
                                  : TelemetryParserState::WaitHeader1;
      }
      break;
    case TelemetryParserState::ReadData:
      data_[dataIndex_++] = byteIn;
      if (dataIndex_ == sizeof(data_))
        state_ = TelemetryParserState::ReadChecksum;
      break;
    case TelemetryParserState::ReadChecksum: {
      uint8_t checksum = 0;
      for (uint8_t dataByte : data_)
        checksum += dataByte;
      state_ = TelemetryParserState::WaitHeader1;

      checksum_attempts_++;
      if (checksum != byteIn) {
        checksum_failures_++;
        if (checksum_failures_ == 1) {
          cerr << "[PARSER CHECKSUM FAIL] attempt#" << checksum_attempts_
               << " expected=" << hex << setfill('0') << setw(2)
               << (int)checksum << " received=" << setw(2) << (int)byteIn << dec
               << endl;
        }
        return false;
      }

      count1 = static_cast<int16_t>((static_cast<uint16_t>(data_[0]) << 8) |
                                    data_[1]);
      count2 = static_cast<int16_t>((static_cast<uint16_t>(data_[2]) << 8) |
                                    data_[3]);
      return true;
    }
    }
    return false;
  }

private:
  TelemetryParserState state_ = TelemetryParserState::WaitHeader1;
  uint8_t data_[4] = {};
  size_t dataIndex_ = 0;
  uint64_t checksum_attempts_ = 0;
  uint64_t checksum_failures_ = 0;
};

void handleTelemetry(const EncoderTelemetry &telemetry) {
#if DEBUG_TELEMETRY
  {
    lock_guard<mutex> lock(consoleMutex);
    cout << fixed << setprecision(6) << "[RX ENCODER] t=" << telemetry.pcTime
         << " count1=" << telemetry.count1 << " count2=" << telemetry.count2
         << " q1=" << telemetry.q1Actual << " q2=" << telemetry.q2Actual
         << '\n';
  }
#endif
}

// ===== TELEMETRY START =====
void startTrajectoryTelemetry() {
  uint64_t newTrajectoryID;
  {
    lock_guard<mutex> lock(telemetryMutex);
    ++trajectoryID;
    newTrajectoryID = trajectoryID;
    telemetryBuffer.clear();
    trajectoryTelemetryStart = trajectoryStart;
    trajectoryStartCount1 = haveActualTelemetry ? latestActualCount1 : 0;
    trajectoryStartCount2 = haveActualTelemetry ? latestActualCount2 : 0;
    telemetryRecording = true;
  }
  lock_guard<mutex> consoleLock(consoleMutex);
  cout << "[TELEMETRY] Recording trajectory " << newTrajectoryID << endl;
}

// ===== TELEMETRY SAVE =====
filesystem::path nextTrajectoryFile(const filesystem::path &directory) {
  error_code directoryError;
  if (!filesystem::exists(directory, directoryError) &&
      !filesystem::create_directories(directory, directoryError) &&
      directoryError) {
    throw runtime_error("Could not create trajectory output directory");
  }

  time_t rawNow = system_clock::to_time_t(system_clock::now());
  tm localTime = {};
  localtime_s(&localTime, &rawNow);

  const string label = directory.filename().string();
  ostringstream baseName;
  baseName << label << "_" << put_time(&localTime, "%Y%m%d_%H%M%S");

  int suffix = 0;
  while (true) {
    ostringstream fileName;
    fileName << baseName.str();
    if (suffix > 0)
      fileName << "_" << suffix;
    fileName << ".csv";

    const filesystem::path candidate = directory / fileName.str();
    if (!filesystem::exists(candidate))
      return candidate;
    ++suffix;
  }
}

bool writeTelemetryFile(const filesystem::path &outputPath,
                        const vector<EncoderTelemetry> &samples,
                        double totalTime, int16_t startCount1,
                        int16_t startCount2, int16_t endCount1,
                        int16_t endCount2, bool includeDesiredCounts,
                        bool requireNewFile) {
  error_code directoryError;
  const filesystem::path outputDirectory = outputPath.parent_path();
  if (!filesystem::exists(outputDirectory, directoryError) &&
      !filesystem::create_directories(outputDirectory, directoryError) &&
      directoryError) {
    lock_guard<mutex> consoleLock(consoleMutex);
    cerr << "[CSV ERROR] Output directory is not accessible: "
         << outputDirectory.string() << endl;
    return false;
  }

  if (requireNewFile && filesystem::exists(outputPath)) {
    lock_guard<mutex> consoleLock(consoleMutex);
    cerr << "[CSV ERROR] Refusing to overwrite existing file "
         << outputPath.string() << endl;
    return false;
  }

  const string filename = outputPath.string();
  ofstream logFile(filename, ios::out | ios::trunc);
  if (!logFile.is_open()) {
    lock_guard<mutex> consoleLock(consoleMutex);
    cerr << "[CSV ERROR] Could not open " << filename << endl;
    return false;
  }

  logFile << "TotalTime,StartCount1,StartCount2,EndCount1,EndCount2\n";
  logFile << fixed << setprecision(9) << totalTime << ',' << startCount1 << ','
          << startCount2 << ',' << endCount1 << ',' << endCount2 << '\n';
  logFile << '\n';
  logFile << "time,desired_count1,desired_count2,actual_count1,"
             "actual_count2,TauDyn1_Nm,TauDyn2_Nm\n";

  for (const EncoderTelemetry &sample : samples) {
    if (includeDesiredCounts) {
      logFile << sample.pcTime << ',' << sample.desiredCount1 << ','
              << sample.desiredCount2 << ',' << sample.count1 << ','
              << sample.count2 << ',' << sample.tau1 << ',' << sample.tau2
              << '\n';
    } else {
      logFile << sample.pcTime << ',' << sample.desiredCount1 << ','
              << sample.desiredCount2 << ',' << sample.count1 << ','
              << sample.count2 << ',' << sample.tau1 << ',' << sample.tau2
              << '\n';
    }
  }

  logFile.flush();
  return true;
}

void stopAndSaveTrajectoryTelemetry() {
  int16_t startCount1;
  int16_t startCount2;
  int16_t endCount1;
  int16_t endCount2;
  vector<EncoderTelemetry> samples;
  {
    lock_guard<mutex> lock(telemetryMutex);
    telemetryRecording = false;
    startCount1 = trajectoryStartCount1;
    startCount2 = trajectoryStartCount2;
    endCount1 = haveActualTelemetry ? latestActualCount1 : startCount1;
    endCount2 = haveActualTelemetry ? latestActualCount2 : startCount2;
    samples = telemetryBuffer;
    telemetryBuffer.clear();
  }

  const double totalTime =
      duration<double>(steady_clock::now() - trajectoryStart).count();
  const filesystem::path directory =
      trajectoryDirectoryFor(activeTrajectoryType);
  filesystem::path trajectoryPath;
  {
    lock_guard<mutex> lock(trajectoryFileMutex);
    trajectoryPath = nextTrajectoryFile(directory);
  }

  const bool trajectorySaved =
      writeTelemetryFile(trajectoryPath, samples, totalTime, startCount1,
                         startCount2, endCount1, endCount2, true, true);

  lock_guard<mutex> consoleLock(consoleMutex);
  if (trajectorySaved) {
    cout << "[CSV] Saved " << trajectoryPath.string() << " (" << samples.size()
         << " samples)" << endl;
  }
}

// Runs separately from command input and trajectory transmission. ReadFile has
// short COM timeouts, and serialMutex covers only the read operation itself.
void telemetryReceiveLoop() {
  TelemetryParser parser;
  bool readErrorReported = false;

  while (serialHandle != INVALID_HANDLE_VALUE) {
    uint8_t rxBuffer[64];
    DWORD bytesRead = 0;
    BOOL readOk;
    {
      lock_guard<mutex> lock(serialMutex);
      readOk = ReadFile(serialHandle, rxBuffer, sizeof(rxBuffer), &bytesRead,
                        nullptr);
    }
    if (!readOk) {
      if (!readErrorReported) {
        cerr << "[PC RX ERROR] ReadFile failed: " << GetLastError() << endl;
        readErrorReported = true;
      }
      this_thread::sleep_for(chrono::milliseconds(2));
      continue;
    }
    for (DWORD i = 0; i < bytesRead; ++i) {
      int16_t count1;
      int16_t count2;
      if (!parser.push(rxBuffer[i], count1, count2))
        continue;

      const double q1Actual = static_cast<double>(count1) * 2.0 * PI /
                              static_cast<double>(pwm_num1);
      const double q2Actual = static_cast<double>(count2) * 2.0 * PI /
                              static_cast<double>(pwm_num2);

      {
        lock_guard<mutex> lock(telemetryMutex);
        latestActualCount1 = count1;
        latestActualCount2 = count2;
        haveActualTelemetry = true;
        if (telemetryRecording) {
          const double trajectoryTime =
              duration<double>(steady_clock::now() - trajectoryTelemetryStart)
                  .count();
          telemetryBuffer.push_back({trajectoryTime, lastDesiredCount1,
                                     lastDesiredCount2, count1, count2,
                                     q1Actual, q2Actual, lastTau1, lastTau2});
          ++telemetryPacketCount;
        }
      }

      EncoderTelemetry telemetry = {
          0.0,      lastDesiredCount1, lastDesiredCount2, count1,  count2,
          q1Actual, q2Actual,          lastTau1,          lastTau2};
      handleTelemetry(telemetry);
    }
  }
}

void solveInverseKinematics(point target) {
  joint theta = {0.0f, 0.0f};

  printf("Target point: (%d, %d, %d)\n", target.x, target.y, target.z);

  if (indata.RRplanar_in(target, theta)) {
    printf("IK solution found\n");
    printf("theta1 = %.4f rad = %.1f deg\n", theta.theta1,
           theta.theta1 * 180.0f / PI);
    printf("theta2 = %.4f rad = %.1f deg\n", theta.theta2,
           theta.theta2 * 180.0f / PI);
  } else {
    printf("Target point is out of robot workspace\n");
  }
}

struct ComPortInfo {
  string comPort;
  string friendlyName;
  string hardwareId;
};

vector<ComPortInfo> enumerateSerialPorts() {
  vector<ComPortInfo> ports;

  HMODULE setupApiModule = LoadLibraryA("setupapi.dll");
  if (setupApiModule == nullptr) {
    cout << "[SERIAL ERROR] Failed to load setupapi.dll" << endl;
    return ports;
  }

  typedef HDEVINFO(WINAPI * PFN_SetupDiGetClassDevsA)(CONST GUID *, PCSTR, HWND,
                                                      DWORD);
  typedef BOOL(WINAPI * PFN_SetupDiEnumDeviceInfo)(HDEVINFO, DWORD,
                                                   PSP_DEVINFO_DATA);
  typedef BOOL(WINAPI * PFN_SetupDiGetDeviceRegistryPropertyA)(
      HDEVINFO, PSP_DEVINFO_DATA, DWORD, PDWORD, PBYTE, DWORD, PDWORD);
  typedef BOOL(WINAPI * PFN_SetupDiDestroyDeviceInfoList)(HDEVINFO);

  PFN_SetupDiGetClassDevsA SetupDiGetClassDevsA_fn =
      reinterpret_cast<PFN_SetupDiGetClassDevsA>(
          GetProcAddress(setupApiModule, "SetupDiGetClassDevsA"));
  PFN_SetupDiEnumDeviceInfo SetupDiEnumDeviceInfo_fn =
      reinterpret_cast<PFN_SetupDiEnumDeviceInfo>(
          GetProcAddress(setupApiModule, "SetupDiEnumDeviceInfo"));
  PFN_SetupDiGetDeviceRegistryPropertyA SetupDiGetDeviceRegistryPropertyA_fn =
      reinterpret_cast<PFN_SetupDiGetDeviceRegistryPropertyA>(
          GetProcAddress(setupApiModule, "SetupDiGetDeviceRegistryPropertyA"));
  PFN_SetupDiDestroyDeviceInfoList SetupDiDestroyDeviceInfoList_fn =
      reinterpret_cast<PFN_SetupDiDestroyDeviceInfoList>(
          GetProcAddress(setupApiModule, "SetupDiDestroyDeviceInfoList"));

  if (SetupDiGetClassDevsA_fn == nullptr ||
      SetupDiEnumDeviceInfo_fn == nullptr ||
      SetupDiGetDeviceRegistryPropertyA_fn == nullptr ||
      SetupDiDestroyDeviceInfoList_fn == nullptr) {
    cout << "[SERIAL ERROR] SetupAPI entry points not found" << endl;
    FreeLibrary(setupApiModule);
    return ports;
  }

  const GUID guidDevClassPorts = {
      0x4D36E978,
      0xE325,
      0x11CE,
      {0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18}};

  HDEVINFO deviceInfoSet = SetupDiGetClassDevsA_fn(&guidDevClassPorts, nullptr,
                                                   nullptr, DIGCF_PRESENT);
  if (deviceInfoSet == INVALID_HANDLE_VALUE) {
    cout << "[SERIAL ERROR] SetupDiGetClassDevsA failed" << endl;
    FreeLibrary(setupApiModule);
    return ports;
  }

  SP_DEVINFO_DATA deviceInfoData = {};
  deviceInfoData.cbSize = sizeof(SP_DEVINFO_DATA);

  for (DWORD index = 0;
       SetupDiEnumDeviceInfo_fn(deviceInfoSet, index, &deviceInfoData);
       ++index) {
    char friendlyName[512] = {};
    DWORD friendlyNameSize = sizeof(friendlyName);
    DWORD friendlyType = 0;
    BOOL gotFriendly = SetupDiGetDeviceRegistryPropertyA_fn(
        deviceInfoSet, &deviceInfoData, SPDRP_FRIENDLYNAME, &friendlyType,
        (PBYTE)friendlyName, friendlyNameSize, &friendlyNameSize);

    char hardwareId[512] = {};
    DWORD hardwareIdSize = sizeof(hardwareId);
    DWORD hardwareType = 0;
    BOOL gotHardware = SetupDiGetDeviceRegistryPropertyA_fn(
        deviceInfoSet, &deviceInfoData, SPDRP_HARDWAREID, &hardwareType,
        (PBYTE)hardwareId, hardwareIdSize, &hardwareIdSize);

    if (gotFriendly) {
      cout << "[SERIAL] Device: " << friendlyName << endl;
    }
    if (gotHardware) {
      cout << "[SERIAL] HardwareID: " << hardwareId << endl;
    }

    string friendly = gotFriendly ? friendlyName : "";
    string hardware = gotHardware ? hardwareId : "";

    if (friendly.empty() && hardware.empty()) {
      continue;
    }

    string comPort = "";
    size_t comPos = friendly.find("COM");
    if (comPos != string::npos) {
      size_t start = comPos + 3;
      size_t end = start;
      while (end < friendly.size() &&
             isdigit(static_cast<unsigned char>(friendly[end]))) {
        ++end;
      }
      if (end > start) {
        comPort = friendly.substr(comPos, end - comPos);
      }
    }

    if (!comPort.empty()) {
      ComPortInfo portInfo;
      portInfo.comPort = comPort;
      portInfo.friendlyName = friendly;
      portInfo.hardwareId = hardware;
      ports.push_back(portInfo);
    }
  }

  SetupDiDestroyDeviceInfoList_fn(deviceInfoSet);
  FreeLibrary(setupApiModule);
  return ports;
}

bool isArduinoMega2560(const ComPortInfo &port) {
  string friendly = port.friendlyName;
  string hardware = port.hardwareId;

  string friendlyUpper = friendly;
  string hardwareUpper = hardware;
  transform(friendlyUpper.begin(), friendlyUpper.end(), friendlyUpper.begin(),
            ::toupper);
  transform(hardwareUpper.begin(), hardwareUpper.end(), hardwareUpper.begin(),
            ::toupper);

  bool hasArduinoVid = hardwareUpper.find("VID_2341") != string::npos;
  bool hasCh340 = hardwareUpper.find("VID_1A86") != string::npos &&
                  hardwareUpper.find("PID_7523") != string::npos;
  bool isCh340Friendly = friendlyUpper.find("USB-SERIAL CH340") != string::npos;

  if (hasArduinoVid || hasCh340 || isCh340Friendly) {
    return true;
  }

  return false;
}

string detectArduinoMega2560() {
  cout << "[SERIAL] Searching for Arduino Mega2560..." << endl;

  vector<ComPortInfo> ports = enumerateSerialPorts();
  if (ports.empty()) {
    cout << "[SERIAL ERROR] No serial ports detected" << endl;
    return "";
  }

  cout << "[SERIAL] Available COM ports:" << endl;
  for (size_t i = 0; i < ports.size(); ++i) {
    if (!ports[i].comPort.empty()) {
      cout << "  " << ports[i].comPort << endl;
    }
  }

  vector<string> candidatePorts;
  for (size_t i = 0; i < ports.size(); ++i) {
    if (!ports[i].comPort.empty() && isArduinoMega2560(ports[i])) {
      candidatePorts.push_back(ports[i].comPort);
    }
  }

  if (candidatePorts.empty()) {
    cout << "[SERIAL ERROR] Arduino Mega2560 not found." << endl;
    return "";
  }

  if (candidatePorts.size() > 1) {
    cout << "[SERIAL ERROR] Multiple Arduino devices found:" << endl;
    for (size_t i = 0; i < candidatePorts.size(); ++i) {
      cout << "  " << candidatePorts[i] << endl;
    }
    cout << "[SERIAL ERROR] Ambiguous device selection. Using first match."
         << endl;
  }

  string selectedPort = candidatePorts[0];
  cout << "[SERIAL] Arduino Mega2560 detected" << endl;
  cout << "[SERIAL] Port = " << selectedPort << endl;
  return selectedPort;
}

bool openSerialPort(const char *portName) {
  serialHandle = CreateFileA(portName, GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                             OPEN_EXISTING, 0, nullptr);
  if (serialHandle == INVALID_HANDLE_VALUE) {
    cout << "[SERIAL ERROR]" << endl;
    cout << "Failed to open " << portName << endl;
    return false;
  }

  DCB dcb = {};
  if (!GetCommState(serialHandle, &dcb)) {
    cout << "[SERIAL ERROR] GetCommState failed" << endl;
    CloseHandle(serialHandle);
    serialHandle = INVALID_HANDLE_VALUE;
    return false;
  }

  dcb.BaudRate = 921600;
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;

  if (!SetCommState(serialHandle, &dcb)) {
    cout << "[SERIAL ERROR] SetCommState failed" << endl;
    CloseHandle(serialHandle);
    serialHandle = INVALID_HANDLE_VALUE;
    return false;
  }

  COMMTIMEOUTS timeouts = {};
  timeouts.ReadIntervalTimeout = 1;
  timeouts.ReadTotalTimeoutConstant = 1;
  timeouts.ReadTotalTimeoutMultiplier = 1;
  timeouts.WriteTotalTimeoutConstant = 10;
  timeouts.WriteTotalTimeoutMultiplier = 10;
  SetCommTimeouts(serialHandle, &timeouts);

  cout << "[SERIAL] Opened " << portName << endl;
  cout << "[SERIAL] Baud = 921600" << endl;
  cout << "[SERIAL] Format = 8N1" << endl;
  return true;
}

// 11-byte command packet:
//   AA 55 J1_MSB J1_LSB J2_MSB J2_LSB TAU1_MSB TAU1_LSB TAU2_MSB TAU2_LSB SUM
// The torque fields are signed int16 with 1 LSB = 0.001 N*m (range +-32.767).
// The checksum is the low 8 bits of the sum of bytes 0..9 (header included).
bool writeSerialPacket(int16_t desiredCount1, int16_t desiredCount2,
                       int16_t torque1Lsb, int16_t torque2Lsb) {
  if (serialHandle == INVALID_HANDLE_VALUE)
    return false;

  lastDesiredCount1 = desiredCount1;
  lastDesiredCount2 = desiredCount2;

  uint8_t frame[11] = {0xAA,
                       0x55,
                       static_cast<uint8_t>((desiredCount1 >> 8) & 0xFF),
                       static_cast<uint8_t>(desiredCount1 & 0xFF),
                       static_cast<uint8_t>((desiredCount2 >> 8) & 0xFF),
                       static_cast<uint8_t>(desiredCount2 & 0xFF),
                       static_cast<uint8_t>((torque1Lsb >> 8) & 0xFF),
                       static_cast<uint8_t>(torque1Lsb & 0xFF),
                       static_cast<uint8_t>((torque2Lsb >> 8) & 0xFF),
                       static_cast<uint8_t>(torque2Lsb & 0xFF),
                       0x00};
  uint16_t sum = 0;
  for (int i = 0; i < 10; ++i)
    sum += frame[i];
  frame[10] = static_cast<uint8_t>(sum & 0xFF);

  lock_guard<mutex> lock(serialMutex);
  DWORD written = 0;
  BOOL result =
      WriteFile(serialHandle, frame, sizeof(frame), &written, nullptr);

  if (result && written == sizeof(frame)) {
    txPacketCount++;
    return true;
  }

  if (!result) {
    cerr << "[SERIAL ERROR] WriteFile failed: " << GetLastError() << endl;
  }
  return false;
}

// Signed fixed-point torque encoding: 1 LSB = 0.001 N*m, saturating at the
// int16 boundary (+-32.767 N*m).
int16_t clampTorqueLsb(float torqueNm) {
  float lsb = torqueNm * 1000.0f;
  if (lsb > 32767.0f)
    lsb = 32767.0f;
  if (lsb < -32768.0f)
    lsb = -32768.0f;
  return static_cast<int16_t>(lround(lsb));
}

void processInputLoop() {
  while (true) {
    string lineText;
    cout << "\n[INPUT] > ";
    if (!getline(cin, lineText))
      continue;
    if (lineText.empty())
      continue;

    char raw[128];
    strncpy(raw, lineText.c_str(), sizeof(raw) - 1);
    raw[sizeof(raw) - 1] = '\0';

    Command localCmd;
    if (!indata.parseCommand(raw, localCmd))
      continue;

    lock_guard<mutex> lock(programMutex);

    switch (localCmd.type) {
    case COUNT:
      totalCommand = localCmd.count;
      loadedCommand = 0;
      executedCommand = 0;
      loadingProgram = true;
      runProgram = false;
      trajectoryActive = false;
      while (!indata.isEmpty()) {
        Command discard;
        indata.dequeueCmd(discard);
      }
      cout << "[CMD] COUNT = " << totalCommand << endl;
      cout << "[PROGRAM] Program size: " << totalCommand << endl;
      break;

    case HOME:
      currentQ1 = 0.0;
      currentQ2 = PI;
      runProgram = false;
      loadingProgram = false;
      trajectoryActive = false;
      while (!indata.isEmpty()) {
        Command discard;
        indata.dequeueCmd(discard);
      }
      cout << "[CMD] HOME" << endl;
      break;

    case POINT_CMD:
    case LINE:
    case CIRCLE:
      if (loadingProgram) {
        if (indata.enqueueCmd(localCmd)) {
          loadedCommand++;
          cout << "[LOAD] " << loadedCommand << "/" << totalCommand << endl;
          if (loadedCommand >= totalCommand) {
            loadingProgram = false;
            runProgram = true;
            cout << "[PROGRAM] Program Loaded" << endl;
            cout << "[PROGRAM] Auto Run Start" << endl;
          }
        } else {
          cout << "[QUEUE] Command queue is full" << endl;
        }
      } else {
        if (indata.enqueueCmd(localCmd)) {
          runProgram = true;
          cout << "[PROGRAM] Direct command queued" << endl;
        }
      }
      break;

    default:
      break;
    }
  }
}

void controlLoop() {
  while (true) {
    {
      lock_guard<mutex> lock(programMutex);
      if (!runProgram) {
        this_thread::sleep_for(chrono::milliseconds(10));
        continue;
      }
    }

    if (!trajectoryActive) {
      Command nextCmd;
      bool gotOne = false;
      {
        lock_guard<mutex> lock(programMutex);
        if (indata.dequeueCmd(nextCmd))
          gotOne = true;
      }

      if (!gotOne) {
        this_thread::sleep_for(chrono::milliseconds(10));
        continue;
      }

      if (nextCmd.type == POINT_CMD) {
        targetPoint = nextCmd.p1;
        cout << "[POINT] Target = (" << targetPoint.x << ", " << targetPoint.y
             << ", " << targetPoint.z << ")" << endl;

        joint ik = {0.0f, 0.0f};
        if (!indata.RRplanar_in(targetPoint, ik)) {
          cout << "[IK ERROR] Target point is unreachable" << endl;
          continue;
        }

        activeJointTrajectory.clear();
        activeUsesJointTrajectory = false;
        startTheta = {static_cast<float>(currentQ1),
                      static_cast<float>(currentQ2)};
        targetTheta = ik;
        cout << "[IK] theta1 = " << targetTheta.theta1
             << " rad = " << targetTheta.theta1 * 180.0f / PI << " deg" << endl;
        cout << "[IK] theta2 = " << targetTheta.theta2
             << " rad = " << targetTheta.theta2 * 180.0f / PI << " deg" << endl;

        double time1 = fabs(targetTheta.theta1 - startTheta.theta1) *
                       TIME1_PER_90 / (PI / 2.0);
        double time2 = fabs(targetTheta.theta2 - startTheta.theta2) *
                       TIME2_PER_90 / (PI / 2.0);

        trajectoryDuration = max(time1, time2);
        cout << "[TRAJECTORY] duration = " << trajectoryDuration << " s"
             << endl;
      } else if (nextCmd.type == LINE || nextCmd.type == CIRCLE) {
        int16_t planned[MAX_PLAN_SAMPLES * 2];
        int plannedCount = 0;
        float plannedDuration = 0.0f;
        bool plannedOk = false;

        if (nextCmd.type == LINE) {
          cout << "[LINE] Start = (" << nextCmd.p1.x << ", " << nextCmd.p1.y
               << ", " << nextCmd.p1.z << ")  End = (" << nextCmd.p2.x << ", "
               << nextCmd.p2.y << ", " << nextCmd.p2.z << ")" << endl;
          plannedOk =
              indata.planLine(nextCmd.p1, nextCmd.p2, planned, MAX_PLAN_SAMPLES,
                              plannedCount, plannedDuration);
        } else {
          circle plannedCircle;
          plannedCircle.p = nextCmd.p1;
          plannedCircle.radius = nextCmd.radius;
          cout << "[CIRCLE] Center = (" << plannedCircle.p.x << ", "
               << plannedCircle.p.y << ")  radius = " << plannedCircle.radius
               << endl;
          plannedOk =
              indata.planCircle(plannedCircle, planned, MAX_PLAN_SAMPLES,
                                plannedCount, plannedDuration);
        }

        if (!plannedOk || plannedCount <= 0) {
          cout << "[IK ERROR] Trajectory is unreachable or invalid" << endl;
          continue;
        }

        activeJointTrajectory.clear();
        activeJointTrajectory.reserve(plannedCount);
        for (int i = 0; i < plannedCount; ++i) {
          joint sample;
          sample.theta1 = static_cast<float>(planned[2 * i]) * 2.0f * PI /
                          static_cast<float>(pwm_num1);
          sample.theta2 = static_cast<float>(planned[2 * i + 1]) * 2.0f * PI /
                          static_cast<float>(pwm_num2);
          activeJointTrajectory.push_back(sample);
        }
        activeUsesJointTrajectory = true;
        startTheta = activeJointTrajectory.front();
        targetTheta = activeJointTrajectory.back();
        trajectoryDuration = max(static_cast<double>(plannedDuration), 0.2);
        cout << "[TRAJECTORY] type = "
             << (nextCmd.type == LINE ? "LINE" : "CIRCLE")
             << "  samples = " << plannedCount
             << "  duration = " << trajectoryDuration << " s" << endl;
      } else {
        cout << "[CMD] Ignored unsupported command" << endl;
        continue;
      }

      if (!waitForActualTelemetry()) {
        cerr << "[MOTION ERROR] No actual encoder telemetry available" << endl;
        continue;
      }

      activeTrajectoryType = nextCmd.type;
      trajectoryStart = steady_clock::now();
      trajectoryActive = true;
      trajectoryInterpolationCycles = 0;
      startTrajectoryTelemetry();

      // Print trajectory start message
      cout << "[TRAJECTORY START]" << endl;
      cout << "  startQ1 = " << setprecision(6) << startTheta.theta1 << endl;
      cout << "  startQ2 = " << setprecision(6) << startTheta.theta2 << endl;
      cout << "  targetQ1 = " << setprecision(6) << targetTheta.theta1 << endl;
      cout << "  targetQ2 = " << setprecision(6) << targetTheta.theta2 << endl;
      cout << "  duration = " << setprecision(4) << trajectoryDuration << endl;
    }

    if (trajectoryActive) {
      auto now = steady_clock::now();
      double elapsed = duration<double>(now - trajectoryStart).count();
      double u = elapsed / max(trajectoryDuration, 1e-6);
      if (u < 0.0)
        u = 0.0;
      if (u > 1.0)
        u = 1.0;

      // Reference joint position for a path parameter u (the same expression is
      // used for the sample itself and for the derivative samples below).
      auto sampleJoint = [&](double uu) -> pair<double, double> {
        if (uu < 0.0)
          uu = 0.0;
        if (uu > 1.0)
          uu = 1.0;
        if (activeUsesJointTrajectory && !activeJointTrajectory.empty()) {
          const int lastIndex =
              static_cast<int>(activeJointTrajectory.size()) - 1;
          int index = static_cast<int>(lround(uu * lastIndex));
          if (index < 0)
            index = 0;
          if (index > lastIndex)
            index = lastIndex;
          return {activeJointTrajectory[index].theta1,
                  activeJointTrajectory[index].theta2};
        }
        double s = 10.0 * pow(uu, 3) - 15.0 * pow(uu, 4) + 6.0 * pow(uu, 5);
        return {
            startTheta.theta1 + (targetTheta.theta1 - startTheta.theta1) * s,
            startTheta.theta2 + (targetTheta.theta2 - startTheta.theta2) * s};
      };

      const auto q0 = sampleJoint(u);
      const double q1 = q0.first;
      const double q2 = q0.second;

      // Reference velocity/acceleration by central differences on the same
      // reference path (consistent for POINT, LINE and CIRCLE, and free of the
      // branch/wrap spikes that analytic IK differentiation would introduce).
      const double hParam = 0.02;
      const auto qPlus = sampleJoint(u + hParam);
      const auto qMinus = sampleJoint(u - hParam);
      const double dt = hParam * max(trajectoryDuration, 1e-6);
      Torque dqRef = {
          static_cast<float>((qPlus.first - qMinus.first) / (2.0 * dt)),
          static_cast<float>((qPlus.second - qMinus.second) / (2.0 * dt))};
      Torque ddqRef = {
          static_cast<float>((qPlus.first - 2.0 * q1 + qMinus.first) /
                             (dt * dt)),
          static_cast<float>((qPlus.second - 2.0 * q2 + qMinus.second) /
                             (dt * dt))};

      // 2R inverse dynamics feedforward for this reference sample.
      const joint qJoint = {static_cast<float>(q1), static_cast<float>(q2)};
      const Torque tauRef = indata.inverseDynamics(qJoint, dqRef, ddqRef);
      lastTau1 = tauRef.t1;
      lastTau2 = tauRef.t2;

      trajectoryInterpolationCycles++;

      currentQ1 = q1;
      currentQ2 = q2;
      const int16_t desiredCount1 = static_cast<int16_t>(
          lround(q1 * static_cast<double>(pwm_num1) / (2.0 * PI)));
      const int16_t desiredCount2 = static_cast<int16_t>(
          lround(q2 * static_cast<double>(pwm_num2) / (2.0 * PI)));
      const int16_t tauLsb1 = clampTorqueLsb(tauRef.t1);
      const int16_t tauLsb2 = clampTorqueLsb(tauRef.t2);
      bool ok =
          writeSerialPacket(desiredCount1, desiredCount2, tauLsb1, tauLsb2);

      if (elapsed >= trajectoryDuration) {
        stopAndSaveTrajectoryTelemetry();
        cout << "[TRAJECTORY END]" << endl;
        cout << "  finalQ1 = " << setprecision(6) << currentQ1 << endl;
        cout << "  finalQ2 = " << setprecision(6) << currentQ2 << endl;
        cout << "  targetQ1 = " << setprecision(6) << targetTheta.theta1
             << endl;
        cout << "  targetQ2 = " << setprecision(6) << targetTheta.theta2
             << endl;
        cout << "  cycles = " << trajectoryInterpolationCycles << endl;
        cout << "  packets = " << txPacketCount << endl;

        cout << "[POINT DONE]" << endl;
        currentQ1 = targetTheta.theta1;
        currentQ2 = targetTheta.theta2;
        trajectoryActive = false;
        activeTrajectoryType = NONE;
        trajectoryInterpolationCycles = 0;
        activeUsesJointTrajectory = false;
        activeJointTrajectory.clear();

        lock_guard<mutex> lock(programMutex);
        executedCommand++;
        cout << "[PROGRAM] executedCommand = " << executedCommand << "/"
             << totalCommand << endl;

        if (executedCommand >= totalCommand) {
          runProgram = false;
          loadingProgram = false;
          cout << "[PROGRAM] FINISHED" << endl;
        }
      }
    }

    this_thread::sleep_for(chrono::milliseconds(10));
  }
}

int main() {
  startPoint = Point = {5, 0, 0};
  currentQ1 = 0.0;
  currentQ2 = PI;

  cout << "[INFO] PC control node started" << endl;
  cout << "[INFO] Use COUNT N then POINT_CMD x y z" << endl;
  trajectoryTelemetryStart = steady_clock::now();

  // Auto-detect Arduino Mega2560 COM port
  string detectedPort = detectArduinoMega2560();

  if (!detectedPort.empty()) {
    // Convert COM port to Windows device path format
    string devicePath = "\\\\.\\" + detectedPort;
    if (!openSerialPort(devicePath.c_str())) {
      cout << "[SERIAL] Port detected but failed to open. Print-only mode."
           << endl;
    }
  } else {
    cout << "[SERIAL] No Mega2560 detected. Print-only mode." << endl;
  }

  thread telemetryThread;
  if (serialHandle != INVALID_HANDLE_VALUE) {
    telemetryThread = thread(telemetryReceiveLoop);
    cout << "[TELEMETRY] Thread started" << endl;
  }
  thread inputThread(processInputLoop);
  thread controlThread(controlLoop);

  inputThread.join();
  controlThread.join();
  if (telemetryThread.joinable())
    telemetryThread.join();
  return 0;
}
