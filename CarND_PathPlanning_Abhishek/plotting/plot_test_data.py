

import numpy as np;
import matplotlib.pyplot as plt

inputfile = "../build/test_data.txt"

data = dict()

# Things to change in the logging file code in c++
# need \r after last data point is written. \n is not enough.
# record data for fsm and obstacles.
#

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

print("Length of data/ Number of lines: ", str(len(lines)))

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
num_trajectories = (len(headers) - 2 ) /2
print("Num of trajectories: " , str(num_trajectories) )

# Plot data for the obstacles and fsm data.
for i in range(1,num_trajectories):
    sig1 = 's' + str(i)
    sig2 = 'd' + str(i)

    plt.plot(data[sig1],data[sig2])
    plt.hold(True)

# create the legend
my_legend = []
for idx in range(1,num_trajectories+1):
    if(idx<=12):
        mystr = "Obs_" +str(idx)
    else:
        mystr = "fsm_" + str(idx-12)

    my_legend.append(mystr )

plt.legend(my_legend)
plt.ylim([-12, 12])
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

