# Christopher Sandoval 13660
# 03/12/2019
# SR5: Textures

class Mtl(object):
    # Al iniciar el objeto se abre el archivo
    def __init__(self, filename):
        # Se abre el archivo
        mtlFile = open(filename)
        # Se serara cada linea del archivo y se guarda en un array
        self.lines = mtlFile.read().splitlines()
        self.materials = {}

        # Se ejecuta el parseo del archivo
        self.parse()

    def parse(self):
        currentMaterial = ''
        # Para cada linea en el array de lineas
        for line in self.lines:
            # Si la linea contiene algo
            if line != "":
                # Se guarda en letter la primera palabra que aparece en cada linea y en data el resto de la linea
                letter, data = line.split(' ', 1)

                if letter == 'newmtl':
                    # Se guarda el nombre del material
                    currentMaterial = data
                if letter == 'Kd':
                    # Se obtienen los datos de los colores en str
                    colors = data.split(' ')
                    # Se convierten a float y se meten al diccionario de materiales
                    self.materials[currentMaterial] = list(map(float, colors))
