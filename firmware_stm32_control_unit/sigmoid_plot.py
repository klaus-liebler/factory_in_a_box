import numpy as np
import matplotlib.pyplot as plt


def sigmoid(x: np.ndarray) -> np.ndarray:
    """Compute sigmoid σ(x)."""
    return 1.0 / (1.0 + np.exp(-x))


def sigmoid_prime(x: np.ndarray) -> np.ndarray:
    """First derivative σ'(x) = σ(x) * (1 - σ(x))."""
    s = sigmoid(x)
    return s * (1.0 - s)


def sigmoid_second(x: np.ndarray) -> np.ndarray:
    """Second derivative σ''(x) = σ'(x) * (1 - 2σ(x))."""
    s = sigmoid(x)
    s_p = s * (1.0 - s)
    return s_p * (1.0 - 2.0 * s)


def main() -> None:
    x = np.linspace(-4, 4, 400)
    y = sigmoid(x)
    y_p = sigmoid_prime(x)
    y_pp = sigmoid_second(x)

    tangent_points = np.linspace(-4, 4, 16)

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(x, y, label="Sigmoid σ(x)", linewidth=2)
    ax.plot(x, y_p, label="σ'(x)", linestyle="--")
    ax.plot(x, y_pp, label="σ''(x)", linestyle=":")

    # Draw tangents to visualize local slope at 16 points
    for x0 in tangent_points:
        y0 = sigmoid(x0)
        slope = sigmoid_prime(x0)
        tangent = y0 + slope * (x - x0)
        ax.plot(x, tangent, color="gray", alpha=0.4, linewidth=0.8)
        ax.scatter([x0], [y0], color="gray", s=12, zorder=5)

    ax.set_title("Sigmoid mit erster und zweiter Ableitung (inkl. 16 Tangenten)")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.grid(True, alpha=0.3)
    ax.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
