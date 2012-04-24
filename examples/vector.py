#coding=utf-8
import math
import collections

_Vector2 = collections.namedtuple('Vector2', 'x y')
_Vector3 = collections.namedtuple('Vector3', 'x y z')


class Vector2(_Vector2):

    default = None

    def __add__(self, other):
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector2(self.x - other.x, self.y - other.y)

    def __mul__(self, other):
        return Vector2(self.x * other, self.y * other)

    def __div__(self, other):
        return Vector2(self.x / other, self.y / other)

    def __truediv__(self, other):
        return Vector2(self.x / other, self.y / other)

    def __floordiv__(self, other):
        return Vector2(self.x // other, self.y // other)

    def __invert__(self):
        return Vector2(self.y, self.x)

    def normalized(self):
        x, y = self
        isnan, isinf = math.isnan, math.isinf
        if isnan(x) or isinf(x):
            return False
        if isnan(y) or isinf(y):
            return False
        return True

    def length(self):
        return math.hypot(*self)

    def displacement(self, co2, v, dt):
        if self is co2:
            return co2

        co1 = self
        vx, vy = co2.x - co1.x, co2.y - co1.y
        try:
            ratio = v * dt / math.hypot(vx, vy)
        except ZeroDivisionError:
            return co2

        if not (0.0 < ratio < 1.0):  # filter NaN, >= 1.0, <=0.0
            return co2
        return Vector2(vx * ratio + co1.x, vy * ratio + co1.y)

    def distance(self, co2):
        if self is co2:
            return 0.0
        co1 = self
        return math.hypot(co1.x - co2.x, co1.y - co2.y)

    def unitize(self, default=None):
        if default is None:
            default = self.default
        vx, vy = self
        dis = math.hypot(vx, vy)
        if dis > 1.0e-10:
            return Vector2(vx / dis, vy / dis)
        return default

    def dotproduct(self, vec):
        return self.x * vec.x + self.y * vec.y

    def rotate(self, center, rad):
        sn, cs = math.sin(rad), math.cos(rad)
        cx, cy = center.x, center.y
        vx, vy = self.x - cx, self.y - cy
        return Vector2((vx * cs - vy * sn) + cx, (vx * sn + vy * cs) + cy)

# initialize class variable default
Vector2.default = Vector2(1.0, 0.0)


class Vector3(_Vector3):

    default = None

    def __add__(self, other):
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, other):
        return Vector3(self.x * other, self.y * other, self.z * other)

    def __div__(self, other):
        return Vector3(self.x / other, self.y / other, self.z / other)

    def __truediv__(self, other):
        return Vector3(self.x / other, self.y / other, self.z / other)

    def __floordiv__(self, other):
        return Vector3(self.x // other, self.y // other, self.z // other)

    def normalized(self):
        x, y, z = self
        isnan, isinf = math.isnan, math.isinf
        if isnan(x) or isinf(x):
            return False
        if isnan(y) or isinf(y):
            return False
        if isnan(z) or isinf(z):
            return False
        return True

    def length(self):
        vx, vy, vz = self
        return math.sqrt(vx * vx + vy * vy + vz * vz)

    def displacement(self, co2, v, dt):
        if self is co2:
            return co2
        co1 = self
        vx, vy, vz = co2.x - co1.x, co2.y - co1.y, co2.z - co1.z
        try:
            ratio = v * dt / math.sqrt(vx * vx + vy * vy + vz * vz)
        except ZeroDivisionError:
            return co2

        if not (0.0 < ratio < 1.0):  # filter NaN, >= 1.0, <=0.0
            return co2
        return Vector3(vx * ratio + co1.x,
                       vy * ratio + co1.y,
                       vz * ratio + co1.z)

    def distance(self, co2):
        if self is co2:
            return 0.0
        co1 = self
        vx, vy, vz = co1.x - co2.x, co1.y - co2.y, co1.z - co2.z
        return math.sqrt(vx * vx + vy * vy + vz * vz)

    def unitize(self, default=None):
        if default is None:
            default = self.default
        vx, vy, vz = self
        dis = math.sqrt(vx * vx + vy * vy + vz * vz)
        if dis > 1.0e-10:
            return Vector3(vx / dis, vy / dis, vz / dis)
        return default

    def dotproduct(self, vec):
        return self.x * vec.x + self.y * vec.y + self.z * vec.z

# initialize class variable default
Vector3.default = Vector3(1.0, 0.0, 0.0)


class Geometry(object):

    def within_angle2(self, pos, center, udir, radiancos):
        if udir.dotproduct((pos - center).unitize()) < radiancos:
            return False
        return True

    def within_circle2(self, pos, center, radius):
        return center.distance(pos) <= radius

    def within_annulus2(self, pos, center, indiameter, exdiameter):
        dis = center.distance(pos)
        if dis > exdiameter:
            return False
        if dis < indiameter:
            return False
        return True

    def within_rectangle2(self, pos, center, udir, vudir, hlen, vhlen):
        fabs = math.fabs
        vec = pos - center
        if fabs(udir.dotproduct(vec)) > hlen:
            return False
        if fabs(vudir.dotproduct(vec)) > vhlen:
            return False
        return True
