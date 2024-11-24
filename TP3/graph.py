from collections import defaultdict
from typing import *
import time

class Aresta:
    def __init__(self, vert_1: int, vert_2:int, capacity:int, fluxo:int, inversa:bool):
        """A classe cria uma aresta que liga vert_1 e vert_2 que tem fluxo e capacidade"""

        self.vert_1 = int(vert_1)
        self.vert_2 = int(vert_2)

        self.capacity = int(capacity)
        self.fluxo = int(fluxo)

        self.inversa = inversa
        self.capacity_residual = 0
        
        self.pointer_original = None
        self.pointer_inversa = None
        
        if inversa:
            self.capacity_residual = self.fluxo
        else:
            self.capacity_residual = self.capacity - self.fluxo
        
    def atualizar(self, bottleneck: int):
        self.fluxo += bottleneck
        if self.inversa == True:
            self.capacity_residual = self.fluxo
        else:
            self.capacity_residual = self.capacity - self.fluxo
            self.pointer_inversa.atualizar(bottleneck)

class Lista_2:
    def __init__(self, arquivo:str, direcionado: bool = True):
        self.arquivo = arquivo

        self.grafo = defaultdict(list) 
        arquivo = open(self.arquivo, 'r')
        self.tamanho = int(arquivo.readline())
        if direcionado == True:
            for linha in arquivo:   #Constrói o grafo residual
                split = linha.split()
                vert_1 = int(split[0])
                vert_2 = int(split[1])
                capacity = int(split[2])
                self.grafo[vert_1].append(Aresta(vert_1, vert_2, capacity, 0, False))
                original = self.grafo[vert_1][-1]
                self.grafo[vert_2].append(Aresta(vert_2, vert_1, capacity, 0, True))
                inversa = self.grafo[vert_2][-1]
                original.pointer_inversa = inversa
                inversa.pointer_original = original
                
        if direcionado == False:
            for linha in arquivo:   #Constrói o grafo residual
                vert_1 = int(linha.split()[0])
                vert_2 = int(linha.split()[1])
                capacity = int(linha.split()[2])
                self.grafo[vert_1].append(vert_1, vert_2, capacity, 0, False)
                self.grafo[vert_2].append(vert_2, vert_1, capacity, 0, True)
                self.grafo[vert_1].append(vert_1, vert_2, capacity, 0, True)
                self.grafo[vert_2].append(vert_2, vert_1, capacity, 0, False)
        arquivo.close

    def bfs(self, inicio: int, fim: int, delta: int):
        iniciador = 1
        explorados = [0] * (self.tamanho + 1) 
        pais = [0] * (self.tamanho + 1) 
        explorados[inicio] = 1
        pais[inicio] = None

        fila = [inicio]
        while fila != []:
            atual = fila.pop()
            vizinhos = []
            if iniciador == 1:
                vizinhos = self.grafo[atual]
                iniciador = 0
            else:
                vizinhos = self.grafo[atual.vert_2]
            for vizinho in vizinhos:
                if explorados[vizinho.vert_2] == 0 and vizinho.capacity_residual >= delta:
                    explorados[vizinho.vert_2] = 1
                    pais[vizinho.vert_2] = atual
                    if vizinho.vert_2 == fim:
                        pais.append(vizinho)
                        return pais
                    fila.append(vizinho)
        return False

    def get_caminho(self, inicio: int, fim: int, delta: int = 1):
        pais = self.bfs(inicio, fim, delta)
        if pais:
            caminho = [pais[-1]]
            atual = pais[fim]
            while atual != inicio:
                caminho.append(atual)
                atual = pais[atual.vert_2]
            return caminho
        else:
            return False

    def get_bottleneck(self, caminho: List[Aresta]):
        bottleneck = caminho[0]
        for aresta in caminho:
            if aresta.capacity_residual < bottleneck.capacity_residual:
                bottleneck = aresta
        return bottleneck

    def aumentar(self, caminho: List[Aresta]):
        bottleneck = self.get_bottleneck(caminho)
        bottleneck_valor = bottleneck.capacity_residual
        for aresta in caminho:
            aresta.atualizar(bottleneck_valor)

    def ford_fulkerson(self, inicio, fim, delta, imprimir):
        if delta == False:
            caminho = self.get_caminho(inicio, fim)
            while caminho:
                self.aumentar(caminho)
                caminho = self.get_caminho(inicio, fim)

        if delta == True:
            capacity = self.get_capacity(inicio)
            delta = int(capacity//2)
            while delta != 1:     
                caminho = self.get_caminho(inicio, fim, delta)
                while caminho:
                    self.aumentar(caminho)
                    caminho = self.get_caminho(inicio, fim, delta)
                delta = int(delta/2)
            caminho = self.get_caminho(inicio, fim)
            while caminho:
                self.aumentar(caminho)
                caminho = self.get_caminho(inicio, fim)
        
        if imprimir==False:
            fluxo = self.get_fluxo(inicio)
            return fluxo
        
        else:
            tempo_leitura = self.imprimir(imprimir)
            fluxo = self.get_fluxo(inicio)
            return fluxo, tempo_leitura
        
    def get_fluxo(self, vertice: int):
        fluxo = 0
        vizinhos = self.grafo[vertice]
        for aresta in vizinhos:
            fluxo += aresta.fluxo
        return fluxo

    def get_capacity(self, vertice: int):
        capacity = 0
        vizinhos = self.grafo[vertice]
        for aresta in vizinhos:
            capacity += aresta.capacity
        return capacity
    
    def imprimir(self, imprimir: str): #Cria o arquivo com o grafo residual
        arquivo = open(imprimir, 'w')
        arquivo.writelines('aresta,vertice 1,vertice 2,fluxo\n')
        grafo = self.grafo
        count=0
        inicio = time.time()
        for vertice in grafo:
            for aresta in grafo[vertice]:
                if aresta.inversa == False:
                    count += 1
                    arquivo.writelines(f'{count},{aresta.vert_1},{aresta.vert_2},{aresta.fluxo} \n')
        fim = time.time()
        tempo=(fim - inicio)
        arquivo.close()
        return tempo

a=Lista_2("TP3/grafo_rf_1.txt", True)
print(a.ford_fulkerson(1,2,True,False))