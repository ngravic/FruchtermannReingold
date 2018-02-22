#!/usr/bin/python

import argparse
import random
import math
import Gnuplot, Gnuplot.funcutils
import time

# Dado un grafo (en formato de listas), aplica el algoritmo de 
# Fruchtermann-Reingold para obtener (y mostrar) un layout
class FruchtermannReingold:
    def __init__(self, title, vertex, edges, iterations, temperature, 
        attractive_force, repulsive_force, speed):
        self.title = title
        self.vertex = vertex
        self.edges = edges
        self.iterations = iterations
        self.attractive_force = attractive_force
        self.repulsive_force = repulsive_force
        self.positions = {}
        self.forces = {}
        self.temperature = temperature
        self.PLOT_WIDTH = 500
        self.PLOT_HEIGHT = 500
        self.plot = None
        self.created = False
        self.speed = speed

    def calculate_attraction_force(self, value):
        return value ** 2 / self.attractive_force

    def calculate_repulsion_force(self, value):
        return self.repulsive_force ** 2 / value

    def init_vortex(self):
        # Initialization of vortex positions in random places
        plot_x = self.PLOT_WIDTH / 2
        plot_y = self.PLOT_HEIGHT / 2
        to_ret = []
        for i in range(0, len(self.vertex)):
            to_ret.append((self.vertex[i], [random.uniform(-plot_x, plot_x), 
                random.uniform(-plot_y, plot_y)]))
        self.positions = dict(to_ret)

    def cool(self):
        return self.temperature * 0.95

    def norm(self, x):
        return math.sqrt(sum(i ** 2 for i in x))

    def sum(self, v1, v2):
        return [x + y for (x, y) in zip(v1, v2)]

    def sub(self, v1, v2):
        return [x - y for (x, y) in zip(v1, v2)]

    def mult(self, v1, scalar):
        return [x * scalar for x in v1]

    def div(self, v1, scalar):
        return [x / scalar for x in v1]

    def algorithm_step(self):
        # Initialization of forces
        for i in range(0, len(self.vertex)):
            f_node = [0.0, 0.0]
            self.forces[self.vertex[i]] = f_node
        # Calculation repulsion forces
        for i in range(len(self.vertex)):
            vortex_1 = self.vertex[i]
            for j in range(i + 1, len(self.vertex)):
                vortex_2 = self.vertex[j]
                delta = self.sub(self.positions[vortex_1], self.positions[vortex_2])
                mod_delta = max(self.norm(delta), 0.02)
                self.forces[vortex_1] = self.sum(self.forces[vortex_1], \
                    self.mult(self.div(delta, mod_delta), \
                    self.calculate_repulsion_force(mod_delta))
                )
                self.forces[vortex_2] = self.sub(self.forces[vortex_2], \
                    self.mult(self.div(delta, mod_delta), \
                    self.calculate_repulsion_force(mod_delta))
                )
        # Calculation attraction forces
        for edge in self.edges:
            delta = self.sub(self.positions[edge[0]], self.positions[edge[1]])
            mod_delta = max(self.norm(delta), 0.02)
            self.forces[edge[0]] = self.sub(self.forces[edge[0]], 
                self.mult(self.div(delta, mod_delta), 
                self.calculate_attraction_force(mod_delta))
            )
            self.forces[edge[1]] = self.sum(self.forces[edge[1]], 
                self.mult(self.div(delta, mod_delta), 
                self.calculate_attraction_force(mod_delta))
            )
        # Update positions
        for vortex in self.vertex:
            disp = self.forces[vortex]
            mod_disp = max(self.norm(disp), 0.02)
            self.positions[vortex] = self.sum(self.positions[vortex], self.mult(
                    self.div(disp, mod_disp), min(mod_disp, self.temperature))
            )
        # Cool
        self.temperature = self.cool()

    def __draw__(self):
        x_s = map(lambda (_, y): y[0], self.positions.items())
        y_s = map(lambda (_, y): y[1], self.positions.items())
        x_range_min = min(x_s)
        x_range_max = max(x_s)
        y_range_min = min(y_s)
        y_range_max = max(y_s)
        
        if self.plot == None:
            self.plot = Gnuplot.Gnuplot()
            self.plot('set title "{}"'.format(self.title))
        self.plot('set xrange [{}:{}]'.format(x_range_min - 100, x_range_max + 100))
        self.plot('set yrange [{}:{}]'.format(y_range_min - 100, y_range_max + 100))
        u = i = 1        
        for vortex in self.vertex:
            self.plot('set object {} circle center {},{} size 5 fc rgb "black"'.format\
                (i, self.positions[vortex][0], self.positions[vortex][1]))
            i += 1
        for edge in self.edges:
            x1 = self.positions[edge[0]][0]
            x2 = self.positions[edge[1]][0]
            y1 = self.positions[edge[0]][1]
            y2 = self.positions[edge[1]][1]
            self.plot('set arrow {} nohead from {},{} to {},{}'.format\
                (u, x1, y1, x2, y2))
            u += 1
        if self.created :
            self.plot('replot')
        else :
            self.plot('unset key')
            self.plot('plot NaN')
            time.sleep(3.0 / self.speed)
        for m in range(1, i):
            self.plot('unset object {}'.format(i))
        for m in range(1,u):
            self.plot('unset arrow {}'.format(m))
        if self.created:
            time.sleep(1.0 / self.speed)
        self.created = True

    def draw(self):
        self.init_vortex()
        self.__draw__()
        for i in range(0, self.iterations):
            self.algorithm_step()
            self.__draw__()
        self.__draw__()
        time.sleep(3 + 4 / (self.speed / 50))

def read_graph_data():
    V = []
    E = []
    print 'Ingrese el titulo del grafo'
    Title = raw_input()
    print 'Ingrese cantidad de vertices'
    n = input()
    print 'Ingrese los nombres de los vertices, separados por un salto de linea.'
    for i in range(n):
        V.append(raw_input().strip())
    print 'Ingrese las aristas, separando los vertices con un espacio y con un\
        salto de linea entre aristas. Al finalizar ingrese un salto de linea\
        vacio.'
    while True:
        try:
            e = raw_input().split()
            if len(e) == 0:
                return (Title, V, E)
            if (e[0] in V and e[1] in V):
                E.append((e[0], e[1]))
            else:
                print 'El nombre del vertice no existe. No se agregara la arista'
        except IndexError:
            print 'Ha ingresado un formato invalido. No se agregara la arista'
        except EOFError:
            return (Title, V, E)

def add_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--iterations', type = int, default = 50,
        help = 'Cantidad de iteraciones (default 100)')
    parser.add_argument('-t', '--temperature', type = int, default = 200,
        help = 'Temperatura para el algoritmo. Afecta al movimiento de los\
        vertices y de la terminacion del algoritmo')
    parser.add_argument('-af', '--attractive-force', type = float, 
        default = 2000.0, help = 'Constante de las fuerzas de atraccion entre\
        2 vertices')
    parser.add_argument('-rf', '--repulsive-force', type = float,
        default = 30.0, help = 'Constante de las fuerzas de atraccion entre\
        2 vertices')
    parser.add_argument('-s', '--speed', type = int, default = 100, 
        help = 'Velocidad de animacion')
    return parser.parse_args()

def main():
    args = add_arguments()
    graph_data = read_graph_data()
    graph = FruchtermannReingold(graph_data[0], graph_data[1], graph_data[2], 
        args.iterations, args.temperature, args.attractive_force,
        args.repulsive_force, args.speed)
    graph.draw()

if __name__ == '__main__':
    main()