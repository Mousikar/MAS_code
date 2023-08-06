from pylab import array, linspace, subplots
from ddeint import ddeint


def model(Y, t, d):
    X = Y(t)
    Xd = Y(t - d)
    return  array([- Xd[0] + Xd[1], 2 * Xd[1] - 3 * Xd[1]])

# def g(t):
#     return 0.8
g = lambda t: array([1,2])
print(g)
tt = linspace(2, 30, 20000)

fig, ax = subplots(1, figsize=(4, 4))

col=['r','b']
lin=['-','--']
count=0
for d in [0, 0.4]:
    print("Computing for d=%.02f" % d)
    yy = ddeint(model, g, tt, fargs=(d,))
    print(yy)
    # WE PLOT X AGAINST Y
    ax.plot(yy[:,0],col[0]+lin[count], lw=2, label="x_1,delay = %.01f" % d)
    ax.plot(yy[:,1],col[1]+lin[count], lw=2, label="x_2,delay = %.01f" % d)
    count+=1
    ax.legend()
ax.figure.savefig("lotka.jpeg")