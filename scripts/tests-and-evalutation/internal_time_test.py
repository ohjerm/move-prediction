import matplotlib.pyplot as plt

# x = [11.84,9.30,5.37,7.59,11.64,8.58,7.14,4.72,5.77,9.1,6.23,4.91,4.19,6.10,10.46,7.62,6.24,4.85,5.25,8.37]
x = [6.03,6.26,4.48,1.64,11.38,1.32,13.28,12.66,5.59,9.22,2.27,2.01,14.13,13.51,3.46,2.82,3.25,7.25,2.51,6.89,3.35,1.18,5.32,13.43,1.25,4.14,6.38,13.88,11.82,1.20]

fig1, ax1 = plt.subplots()
ax1.set_title('Time(s) to reach bottle')
dic = ax1.boxplot(x)

for line in dic['boxes']:
    print(line.get_ydata())
    
for line in dic['medians']:
    print(line.get_ydata())
    
for line in dic['whiskers']:
    print(line.get_ydata())
    
# results: 4.19, 5.34, 6.69, 8.71, 11.84
# results round 2: 1.18, 2.59, 5.46, 10.84, 14.13