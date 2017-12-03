import numpy as np
import shapely.geometry
import shapely.prepared

class OccupancyMap:

    def __init__(self, nx, xmin, xmax, ny, ymin, ymax):

        self.nx = nx
        self.xmin = xmin
        self.xmax = xmax
        self.ny = ny
        self.ymin = ymin
        self.ymax = ymax

        self.occupancy_px = np.zeros((self.nx, self.ny), dtype=np.bool)
        self.occupancy_geom = None

        self.px_points = [None]*nx*ny
        dx = (xmax-xmin) / nx
        dy = (ymax-ymin) / ny
        for i in range(nx):
            for j in range(ny):
                px_points[i+nx*j] = shapely.geometry.Point((xmin+i*dx,ymin+j*dy))

    def add_polyline(points, thickness):
        
        ls = shapely.geometry.LineStrings(points).buffer(thickness)
        self.occupancy_geom = self.occupancy_geom.union(ls)
        prepared_ls = shapely.prepared.prep(ls)
        interior = map(prepared_polygon.contains, self.py_points)
        ls_occupancy_px = np.array(interior, dtype=np.bool).reshape((self.nx,self.ny))
        self.occupancy_px = np.logical_or(self.occupancy_px, ls_occupancy_px)


class Agent:

    def __init__(self, X0, Xinit):

        self.X0 = X0
        self.X = Xinit
        self.t = 0
        self.world_size = 100
        self.world_size_px = 1000
        self.occupancy = OccupancyMap(self.world_size_px, 0, self.world_size = 100, \
                                      self.world_size_px, 0, self.world_size = 100, )

    def dXdt(self, X, u):

        dXdt0 = X[2:4]
        dXdt1 = self.k*(X[0:2]-self.X0) + u
        return np.append(dXdt0, dXdt1)

    def draw_thick_line(X0, X1, thickness):

        if np.array_equal(X0, X1):
            return np.zeros((self.world_size,self.world_size), dtype=np.bool)
        else:
            T = (X1-X0) / np.linalg.norm(X1-X0)
            N = np.array( [T[0], -T[1]] )
            X00 = X0 + thickness/2. * (N-T)
            X11 = X1 + thickness/2. * (T-N)
            return skimage.draw.rectangle(start=X00, end=X11, shape=[self.world_size, self.world_size])


    def move(self, u, dt):

        nPoints = 10
        t = np.linspace(self.t, self.t+dt, nPoints, endpoint=True);
        def dXdt(X, t):
            return self.dXdt(X,u)
        trajectory = scipy.integrate.odeint(dXdt, self.X, t)
        self.t += dt
        self.X = trajectory[nPoints-1]
