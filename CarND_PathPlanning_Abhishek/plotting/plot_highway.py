import numpy as np
import matplotlib.pyplot as plt
# from scipy import interpolate

import csv



inputfile = "highway_csvfile.csv"

data = dict()

# Things to change in the logging file code in c++
# need \r after last data point is written. \n is not enough.
# record data for fsm and obstacles.
#
def fit_spline(x_points,y_points):
    fit = interpolate.splrep(x_points,y_points)

    return fit

def eval_spline(x,fit):
    y = interpolate.splev(x,fit)

    return y


with open(inputfile) as f:
    lines = f.readlines()
    #get headers
    headers = lines[0].split()
    print( "Total columns in Headers: %d" % len(headers))
    header_list = '[%s]' % ', '.join(map(str, headers))
    print (header_list)

    # remove first line, since that is the header
    lines = lines[1:]

# Create empty lists for all headers.
for idx, header in enumerate(headers):
    data[header] = []

print("Length of data/ Number of lines: " + str(len(lines)))

# Iterate over all lines of data
for ll,line in enumerate(lines):
    # print("ll =", str(ll) )
    tokens = line.split()

    # Copy data only for lines that have exactly the same data as the number of headers .
    if(len(tokens)==len(headers)):

        # print(len(tokens))
        for idx,header in enumerate(headers):
            # print("idx =", str(idx))
            data[header].append(float(tokens[idx]))
    else:
        print("Skipping line no: %i.. not enough data points for all headers" % ll)

# Determine the number of values to plot.
print(data.keys())

plt.plot(data['x'],data['y'],'x-')
plt.grid('on')
plt.xlabel('x-meter')
plt.ylabel('y-meter')
plt.title(" Global x vs y values for the highway map")
plt.show()

#     for idx,header in enumerate(headers):
#         newarray = []
#         # print("idx : " + str(idx))
#         for line in lines:
#             # print ('[%s]' % ', '.join(map(str, line.split())))
#             newarray.append(float(line.split(' ')[idx]))
#
#         # Add this to the dictionary
#         data[header] = np.array(newarray)
#
# print((data['d5']))
#
# plt.plot(data['s5'],data['d5'])
# plt.show()

