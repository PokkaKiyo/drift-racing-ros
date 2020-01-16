import math
import matplotlib.pyplot as plt

def calculate_y2(i):
    x2 = i * i
    x4 = x2 * x2
    a2 = 10 * 10

    y2 = x2 - (x4 / a2)
    return y2

def get_figure_eight_coordinates():
    x, y = [], []

    for i in range(0, 11):
        x.append(float(i))
        y2 = calculate_y2(i)
        y.append(math.sqrt(y2))

    for i in range(9, -1, -1):
        x.append(float(i))
        y2 = calculate_y2(i)
        y.append(-1 * math.sqrt(y2))

    for i in range(1, 11):
        x.append(-1.0 * i)
        y2 = calculate_y2(i)
        y.append(math.sqrt(y2))

    for i in range(9, 0, -1):
        x.append(-1.0 * i)
        y2 = calculate_y2(i)
        y.append(-1 * math.sqrt(y2))
    
    x.append(0)
    y.append(0)

    plt.plot(x, y)
    plt.show()
    exit()

    if len(x) != len(y):
        return None

    coordinates = []

    for i in range(len(x)):
        coordinates.append((x[i], y[i]))
    print(coordinates)
    return coordinates
