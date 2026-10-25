import torch
import torch.nn as nn

class LSTMSequentialModel(nn.Module):
    def __init__(self, input_size=1, hidden_size=50, num_layers=2):
        """
        Hàm khởi tạo kiến trúc mô hình LSTM tuần tự.
        Tham số:
        - input_size (int): Số lượng đặc trưng (features) của dữ liệu tại mỗi bước thời gian (time step).
                            Ví dụ = 1 nếu chỉ dự đoán dựa trên một chuỗi giá đóng cửa duy nhất.
        - hidden_size (int): Số lượng nút ẩn (hidden units/neurons) trong mỗi lớp LSTM.
                             Số lượng càng lớn, mô hình học được quan hệ càng phức tạp nhưng tốn tài nguyên hơn.
        - num_layers (int): Tổng số lớp LSTM định nghĩa ở tham số mặc định (trong bài này ta tự tách thủ công thành 2 lớp riêng biệt).
        """
        super(LSTMSequentialModel, self).__init__()
        
        # --- LỚP LSTM THỨ NHẤT & DROPOUT 1 ---
        # Nhận đầu vào ban đầu và trích xuất đặc trưng chuỗi thời gian sang không gian ẩn (hidden_size)
        self.lstm1 = nn.LSTM(
            input_size=input_size,    # Kích thước số lượng đặc trưng đầu vào (mặc định = 1)
            hidden_size=hidden_size,  # Kích thước không gian đặc trưng đầu ra của lớp này (mặc định = 50)
            num_layers=1,             # Số lớp LSTM chồng bên trong cấu hình này (thiết lập = 1)
            batch_first=True          # Định dạng dữ liệu đầu vào: (batch_size, sequence_length, features)
        )
        # Giảm overfitting bằng cách tắt ngẫu nhiên 20% các kết nối trong quá trình huấn luyện
        self.dropout1 = nn.Dropout(p=0.2)
        
        # --- LỚP LSTM THỨ HAI & DROPOUT 2 ---
        # Tiếp tục xử lý sâu hơn chuỗi đặc trưng nhận được từ lớp LSTM thứ nhất
        self.lstm2 = nn.LSTM(
            input_size=hidden_size,   # Kích thước đầu vào chính là kích thước đầu ra của lstm1 (mặc định = 50)
            hidden_size=hidden_size,  # Kích thước đầu ra của trạng thái ẩn lớp này (mặc định = 50)
            num_layers=1,             # Số lớp LSTM chồng bên trong cấu hình này (thiết lập = 1)
            batch_first=True          # Giữ nguyên định dạng dữ liệu có chiều batch đi đầu tiên
        )
        # Lớp Dropout thứ hai ngăn ngừa overfitting ở tầng sâu hơn
        self.dropout2 = nn.Dropout(p=0.2)
        
        # --- CÁC LỚP KẾT NỐI ĐẦY ĐỦ (LINEAR/DENSE LAYERS) ---
        # Chuyển đổi các đặc trưng học được từ LSTM về kết quả dự báo mong muốn
        self.fc1 = nn.Linear(
            in_features=hidden_size,  # Kích thước đầu vào nhận từ đầu ra của lớp lstm2 (mặc định = 50)
            out_features=25           # Hạ chiều dữ liệu xuống còn 25 nút ẩn ở tầng trung gian
        )
        self.fc2 = nn.Linear(
            in_features=25,           # Kích thước đầu vào nhận từ tầng trung gian fc1
            out_features=1            # Đầu ra cuối cùng bằng 1 (thường dùng dự báo 1 giá trị số cho bài toán Regression)
        )

    def forward(self, x):
        out, _ = self.lstm1(x)
        out = self.dropout1(out)

        out, _ = self.lstm2(out)
        out = self.dropout2(out)

        out = out[:, -1, :]
        out = torch.relu(self.fc1(out))

        out = self.fc2(out)
        return out