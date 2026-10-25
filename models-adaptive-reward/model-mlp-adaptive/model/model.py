
import torch
import torch.nn as nn


class RewardUtilityMLP(nn.Module):
    """
    MLP dự đoán utility của một hành động tinh chỉnh reward.
    Input:
        state:  Tensor [batch_size, state_dim]
        action: Tensor [batch_size, action_dim]
    Output:
        utility: Tensor [batch_size, 1]
    """
    def __init__(
        self,
        state_dim: int = 40,
        action_dim: int = 4,
        hidden_dims: tuple[int, ...] = (64, 32),
    ):
        super().__init__()

        input_dim = state_dim + action_dim
        layers = []
        prev_dim = input_dim

        for hidden_dim in hidden_dims:
            layers.extend([
                nn.Linear(prev_dim, hidden_dim),
                nn.ReLU(),
            ])
            prev_dim = hidden_dim

        layers.append(nn.Linear(prev_dim, 1))

        self.network = nn.Sequential(*layers)

    def forward(
        self,
        state: torch.Tensor,
        action: torch.Tensor,
    ) -> torch.Tensor:
        x = torch.cat([state, action], dim=-1)
        return self.network(x)


if __name__ == "__main__":
    model = RewardUtilityMLP()

    batch_size = 8
    state = torch.randn(batch_size, 40)
    action = torch.randn(batch_size, 4)

    utility = model(state, action)

    print(model)
    print("Output shape:", utility.shape)
    # Expected: torch.Size([8, 1])
