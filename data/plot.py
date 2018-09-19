import matplotlib.pyplot as plt
import csv

def read_file(file):
    curr_x = 0
    x = []
    y = []
    with open(file, 'r') as f:
        reader = csv.reader(f, delimiter=';')
        for row in reader:
            x.append(curr_x)
            y.append(float(row[0]))
            curr_x += 1
    return x, y


x, y = read_file('radar.csv')
plt.plot(x, y, label='Radar')

x, y = read_file('laser.csv')
plt.plot(x, y, label='Laser')

plt.plot([0, x[-1]], [7.815, 7.815], label="7.815")
plt.plot([0, x[-1]], [5.991, 5.991], label="5.991")

plt.legend()
plt.show()
