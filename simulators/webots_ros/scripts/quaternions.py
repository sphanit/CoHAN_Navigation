import numpy as np
import math

class Quaternions():
    def euler_to_quaternion(self, angles):
        roll, pitch, yaw = map(math.radians, angles)  # Convertir los ángulos a radianes
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return [qw, qx, qy, qz]

    def conjugate_quaternion(self, quaternion):
        qw, qx, qy, qz = quaternion
        conjugate = [qw, -qx, -qy, -qz]
        return self.normalize_quaternion(conjugate)
    
    def normalize_quaternion(self, quaternion):
        d = quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]
        if d == 0:
            return [1, quaternion[0], quaternion[1], quaternion[2]]
        if abs(1 - d) < 2.107342e-08:
            d = 2 / (1 + d)
        else:
            d = 1 / math.sqrt(d)

        return [round(quaternion[0] * d, 6), round(quaternion[1] * d, 6), round(quaternion[2] * d, 6), round(quaternion[3] * d, 6)]

    def multiply_quaternions(self, quaternion1, quaternion2):
        w1, x1, y1, z1 = quaternion1
        w2, x2, y2, z2 = quaternion2
        
        qw = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        qx = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        qy = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        qz = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        
        return [qw, qx, qy, qz]
    
    def axis_angle_to_quaternion(self, axis, angle):
        zero = self.quaternion_zero()
        l = axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]
        if l > 0:
            axis = np.array(axis)
            angle /= 2.0
            l = math.sin(angle) / math.sqrt(l)
            w = math.cos(angle)
            x = axis[0] * math.sin(l)
            y = axis[1] * math.sin(l)
            z = axis[2] * math.sin(l)

            return [round(w, 6), round(x, 6), round(y, 6), round(z, 6)]
        else:
            return zero
    
    def quaternion_to_axis_angle(self, quaternion):
        axis_angle = [0, 0, 0, 0]
        quaternion = np.array(quaternion)
        w, x, y, z = quaternion
        if w > 1:
            w, x, y, z = self.normalize_quaternion(quaternion)
        if w <= -1:
            axis_angle[3] = 2 * math.pi
        elif w < 1:
            axis_angle[3] = 2 * math.acos(w)  # Calcular el ángulo en radianes
        else:
            axis_angle[3] = 0

        if axis_angle[3] < 0.0001:
            return [0, 1, 0, 0]

        inv = 1 / math.sqrt(x * x + y * y + z * z)
        # print("quaternion_to_axis_angle", [x * inv, y * inv, z * inv, angle])
        axis_angle[0] = round(x * inv, 6)
        axis_angle[1] = round(y * inv, 6)
        axis_angle[2] = round(z * inv, 6)
        axis_angle[3] = round(axis_angle[3], 6)
        return axis_angle
        
    def quaternion_zero(self):
        return [1, 0, 0, 0]
    
    def test(self):
        print("e")
        orientaciones_python = [
            [1, 0, 0, 0.5236],
            [0, 1, 0, 0.7854],
            [0, 0, 1, 1.0472],
            [1, 1, 1, 0.8727],
            [-1, 0, 0, 1.5708],
            [0, -1, 0, 2.0944],
            [0, 0, -1, 2.6179],
            [1, 1, -1, 1.2217],
            [-1, -1, -1, 1.0472],
            [1, 0, 1, 0.5236],
            [-1, 0, -1, 0.7854],
            [0, 1, 1, 1.0472],
            [1, 1, 0, 1.5708],
            [-1, -1, 0, 2.0944],
            [0, -1, -1, 2.6179],
            [1, -1, -1, 1.2217],
            [-1, 1, 1, 1.0472],
            [1, 0, -1, 0.5236],
            [-1, 0, 1, 0.7854],
            [0, 1, -1, 1.0472]
        ]

        cuaterniones = [
    [1, 0, 0, 0],        # Cuaternión 1: Identidad
    [0, 1, 0, 0],        # Cuaternión 2: Rotación en el eje x
    [0, 0, 1, 0],        # Cuaternión 3: Rotación en el eje y
    [0, 0, 0, 1],        # Cuaternión 4: Rotación en el eje z
    [0.5, 0.5, 0.5, 0.5],   # Cuaternión 5: Rotación diagonal
    [0.707, 0, 0, 0.707],   # Cuaternión 6: Rotación de 45 grados en el eje y
    [0.866, 0.5, 0, 0],     # Cuaternión 7: Rotación de 60 grados en el plano xy
    [0, 0.707, 0.707, 0],   # Cuaternión 8: Rotación de 90 grados en el plano yz
    [0.5, 0, 0.5, 0.707],   # Cuaternión 9: Rotación de 120 grados en el plano zx
    [0.258, 0.965, 0, 0],   # Cuaternión 10: Rotación de 150 grados en el eje x
    [0, 0, 0.258, 0.965],   # Cuaternión 11: Rotación de 150 grados en el eje y
    [0.707, 0, 0.707, 0],   # Cuaternión 12: Rotación de 90 grados en el plano xy
    [0, 0.866, 0, 0.5],     # Cuaternión 13: Rotación de 60 grados en el plano yz
    [0.5, 0.5, 0.5, -0.5],  # Cuaternión 14: Rotación diagonal negativa
    [0.866, -0.5, 0, 0],    # Cuaternión 15: Rotación de -60 grados en el plano xy
    [0, 0.707, -0.707, 0],  # Cuaternión 16: Rotación de -90 grados en el plano yz
    [0.5, 0, 0.5, -0.707],  # Cuaternión 17: Rotación de -120 grados en el plano zx
    [0.258, -0.965, 0, 0],  # Cuaternión 18: Rotación de -150 grados en el eje x
    [0, 0, -0.258, -0.965], # Cuaternión 19: Rotación de -150 grados en el eje y
    [-0.5, -0.5, -0.5, -0.5] # Cuaternión 20: Rotación en el eje opuesto a la identidad
]
        for i, cuaternion in enumerate(orientaciones_python):
            print("QUATERNION", i)
            # print("conjugate_quaternion", self.conjugate_quaternion(cuaternion))
            # print("quaternion_to_axis_angle", self.quaternion_to_axis_angle(cuaternion))
            # if i < len(cuaterniones)-1:
            #     print("multiply_quaternions", self.multiply_quaternions(cuaterniones[i], cuaterniones[i+1]))
            # print("quaternion_to_axis_angle", self.quaternion_to_axis_angle(cuaternion))
            print(self.axis_angle_to_quaternion([cuaternion[0], cuaternion[1], cuaternion[2]], cuaternion[3]))

    def __init__(self):
        self.test()

if __name__ == '__main__':
    Quaternions()