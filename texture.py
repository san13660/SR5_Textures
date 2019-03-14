# Christopher Sandoval 13660
# 03/12/2019
# SR5: Textures

import struct

def color(r, g, b):
    return bytes([b, g, r])

class Texture(object):
    def __init__(self, path):
        self.path = path
        self.read()

    # Se abre el archivo de textura
    def read(self):
        img = open(self.path, "rb")
        # Se leen los parametros mas relevantes del header
        img.seek(10)
        header_size = struct.unpack("=l", img.read(4))[0]
        img.seek(18)
        self.width = struct.unpack("=l", img.read(4))[0]
        self.height = struct.unpack("=l", img.read(4))[0]
        self.pixels = []
        img.seek(header_size)

        # Se crea una matriz con todos los pixeles de la imagen
        for y in range(self.height):
            self.pixels.append([])
            for x in range(self.width):
                b = ord(img.read(1))
                g = ord(img.read(1))
                r = ord(img.read(1))

                # Se hace un array con los tres colores de cada pixel
                color=[]
                color.append(r)
                color.append(g)
                color.append(b)
                # Se agrega a la matriz de pixeles
                self.pixels[y].append(color)

        img.close()

    # Sirve para obtener el color de un pixel en especifico
    def get_color_at_pos(self, x, y):
        xr = int(x)
        yr = int(y)
        return self.pixels[yr][xr]