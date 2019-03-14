# Christopher Sandoval 13660
# 03/12/2019
# SR5: Textures

class Obj(object):
    # Al iniciar el objeto se abre el archivo
    def __init__(self, filename):
        # Se abre el archivo
        objFile = open(filename)
        # Se serara cada linea del archivo y se guarda en un array
        self.lines = objFile.read().splitlines()

        # Se declaran los arrays
        self.vertices = []
        self.faces = []
        self.materialNames = []
        self.materialIndex = []
        self.tVertices = []

        # Se ejecuta el parseo del archivo
        self.parse()

    def parse(self):
        faceCount = 0
        # Para cada linea en el array de lineas
        for line in self.lines:
            # Si la linea contiene algo
            if line != "":
                # Se guarda en letter la primera letra que aparece en cada linea y en data el resto de la linea
                letter, data = line.split(' ', 1)

                # Si la primera letra de la linea era v
                if letter == 'v':
                    vertex = []
                    # Dividir cada numero en un array e iterar en cada numero
                    for v in data.split(' '):
                        # Meter el valor float de cada numero en un array temporal
                        vertex.append(float(v))
                    # Meter el array con tres numeros float en el array de vertices
                    self.vertices.append(vertex)

                # Si la primera letra de la linea era vt
                elif letter == 'vt':
                    vertex = []
                    # Dividir cada numero en un array e iterar en cada numero
                    for v in data.split(' '):
                        # Meter el valor float de cada numero en un array temporal
                        vertex.append(float(v))
                    # Meter el array con tres numeros float en el array de vertices de texturas
                    self.tVertices.append(vertex)

                # Si la primera letra era f
                elif letter == 'f':
                    faceCount += 1
                    face = []
                    # Separar por cada espacio
                    for a in data.split(' '):
                        face2 = []
                        # Separar por cada slash
                        for b in a.split('/'):
                            # Si el numero no existe poner 0
                            if b == '':
                                b = 0
                            # Meter el valor int del numero de vertice al array temporal
                            face2.append(int(b))
                        face.append(face2)
                    # Meter el array con todos los numeros de vertices al array de faces
                    self.faces.append(face)

                # Si se indica un cambio de material en las caras siguientes
                elif letter == 'usemtl':
                    # Se guarda la informacion para saber que las siguientes caras usaran el material con este nombre
                    self.materialNames.append(data)
                    self.materialIndex.append(faceCount)