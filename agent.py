import numpy as np
import shapely.geometry
import shapely.prepared
from scipy.integrate import odeint
import matplotlib.pyplot as plt

class Trajectory:

    def __init__(self, xmin, xmax, ymin, ymax, dpi, X0, thickness):

        self.nx = int(dpi*(xmax-xmin))
        self.xmin = xmin
        self.xmax = xmax
        self.ny = int(dpi*(ymax-ymin))
        self.ymin = ymin
        self.ymax = ymax
        self.thickness = thickness

        self.centroid_geom = shapely.geometry.Point(X0[0:2])
        self.occupancy_geom = self.centroid_geom.buffer(self.thickness)
        self.occupancy_px = np.zeros((self.nx, self.ny), dtype=np.bool)

        self.px_points = [None]*self.nx*self.ny
        dx = (xmax-xmin) / self.nx
        dy = (ymax-ymin) / self.ny
        for i in range(self.nx):
            for j in range(self.ny):
                self.px_points[i+self.nx*j] = shapely.geometry.Point((xmin+i*dx,ymin+j*dy))

    def add_polyline(self, points):

        ls = shapely.geometry.LineString(points)
        ls_thick = ls.buffer(self.thickness)
        self.centroid_geom = self.centroid_geom.union(ls)
        self.occupancy_geom = self.occupancy_geom.union(ls_thick)
        prepared_ls_thick = shapely.prepared.prep(ls_thick)
        interior = list(map(prepared_ls_thick.contains, self.px_points))
        ls_occupancy_px = np.array(interior, dtype=np.bool).reshape((self.nx,self.ny))
        self.occupancy_px = np.logical_or(self.occupancy_px, ls_occupancy_px)

    def plot_px(self):

        plt.imshow(self.occupancy_px)
        plt.show()

class Agent:

    def __init__(self, X0, Xinit):

        self.X0 = X0
        self.X = Xinit
        self.t = 0
        self.trajectory = Trajectory(0, 100, 0, 100, 10, X0[0:2], 5)
        self.k = 1

    def dXdt(self, X, u):

        dXdt0 = X[2:4]
        dXdt1 = - self.k*(X[0:2]-self.X0) + u
        return np.append(dXdt0, dXdt1)

    def move(self, u, dt):

        nPoints = 100
        t = np.linspace(self.t, self.t+dt, nPoints, endpoint=True);
        def dXdt(X, t):
            return self.dXdt(X,u)
        new_trajectory = odeint(dXdt, self.X, t)
        self.t += dt
        self.X = new_trajectory[nPoints-1,:]
        self.trajectory.add_polyline(new_trajectory[:,0:2])

agent = Agent((50,50), (50, 70, 15, 0))
agent.move((1, 0), 10)
agent.trajectory.plot_px()
