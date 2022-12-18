import matplotlib.pyplot as plt
X,Y,Z,Yaw,Time = [],[],[],[],[]
with open('test_controller/data.txt') as f:
    for line in f:    
        dt = [ float(x) for x in line.split(',') ]
        X.append(dt[0])
        Y.append(dt[1])
        Z.append(dt[2])
        Yaw.append(dt[3])
        Time.append(dt[4])
plt.plot(Time,X, label = "x")
plt.plot(Time,Y, label = "y")
plt.plot(Time,Z, label = "z")
plt.plot(Time,Yaw, label = "Yaw")
plt.legend()
plt.show()