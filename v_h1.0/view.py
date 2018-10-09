import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import csv
mpl.rcParams['legend.fontsize'] = 10
mpl.rcParams['figure.figsize'] = 8, 6


x=[]
y=[]
z=[]

a=[]
b=[]
c=[]

e=[]
f=[]
g=[]

p=[]
q=[]
r=[]

l=[]
m=[]
n=[]

i=[]
j=[]
k=[]

with open('augmented.csv') as csvfile:
    readCSV = csv.reader(csvfile, delimiter=',')
    next(readCSV)
    for row in readCSV:
        x.append(float(row[0]))
        y.append(float(row[1]))
        z.append(float(row[3]))

with open('30.csv') as csvfile:
    readCSV = csv.reader(csvfile, delimiter=',')
    for row in readCSV:
        a.append(float(row[0]))
        b.append(float(row[1]))
        c.append(float(row[3]))


with open('ends.csv') as csvfile:
    readCSV = csv.reader(csvfile, delimiter=',')
    for row in readCSV:
        e.append(float(row[0]))
        f.append(float(row[1]))
        g.append(float(row[3]))
	
        
with open('end.csv') as csvfile:
    readCSV = csv.reader(csvfile, delimiter=',')
    for row in readCSV:
        p.append(float(row[0]))
        q.append(float(row[1]))
        r.append(float(row[3]))


with open('c2centres.csv') as csvfile:
    readCSV = csv.reader(csvfile, delimiter=',')
    for row in readCSV:
        l.append(float(row[0]))
        m.append(float(row[1]))

with open('c1centres.csv') as csvfile:
    readCSV = csv.reader(csvfile, delimiter=',')
    for row in readCSV:
        i.append(float(row[0]))
        j.append(float(row[1]))


plt.plot(a, b, 'r', label="air-track")

plt.plot(x, y, 'b', label="ground-track")

plt.plot(e, f, '.b')

plt.plot(p,q, '.r')

#plt.plot(l,m, ',k')

#plt.plot(i,j, ',g')

plt.plot(-73.8571,40.7721,'Xk', label="Runway")



legend = plt.legend(loc='upper right', shadow=True)
plt.axis('equal')
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title('Flight path')
#plt.show()

fig = plt.figure()
axyz = fig.gca(projection='3d')
plt.axis('equal')


axyz.axes.get_yaxis().set_ticks([])

axyz.plot(x, y, z, label='ground-track')

axyz.plot(a, b, c, label='air-track')

axyz.scatter(e,f,g, marker='o',  color='r')

axyz.scatter(p,q,r, marker='o',  color='b')

axyz.view_init(elev=3, azim=-110)

axyz.legend()

#axyz.set_xlabel('Longitude')
#axyz.set_ylabel('Latitude')
axyz.set_zlabel('Altitude(in feet)')
axyz.set_xticklabels([])
axyz.set_yticklabels([])
plt.show()
