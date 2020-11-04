#utils.py
import csv
import numpy as np

def save_model_data(path, t, y):
    output_str = "#t, x, y, phi"

    for i in range(0, len(t)):
        output_str += f"\n{t[i]:.4f}"

        for j in range(0, len(y)):
            output_str +=f",{y[j][i]:.4f}"

    with open(path, "x") as file:
        file.write(output_str)

    return True

def read_model_data(path):
    t = []
    y = []
    with open(path, "r") as file:
        reader = csv.reader(file, delimiter=",")
        header = next(reader)

        # TODO: enhance append operation, right now it's probably holding two
        # copies of t and y matrices, both with huge amount of data. Notice that the y matrix is multidimensional

        y =[[] for i in range(0, len(header) - 1)]

        for row in reader:
            t = np.append(t, float(row[0]))

            for i in range(0, len(y)):
                y[i] = np.append(y[i], float(row[i+1]))

    return t, y

