class Edge:
    def __init__(self, s, t, w, r = False): #source, target, weight, reverse
        self.s = s  #int
        self.t = t  #int
        self.w = w  #int
        self.reversivel = True  #Esse atributo será falso se já foi criada uma aresta reversa
                                #a esta no grafo residual
        self.r = r  #bool
        self.f = 0  #fluxo





class Grafo:
    def __init__(self, g, D = True):   #string com path do arquivo, direcionado
        self.n = 0
        self.lista = {}
        self.D = D
        self.visitado = []  #vetor auxiliar para DFS
        self.pai = []
        
        with open(g,'r') as grafo:
            self.n = int(grafo.readline().strip())
            for i in range(1 + self.n):
                self.lista[i] = []
            for linha in grafo.readlines():
                linha = linha.split()
                self.adiciona_aresta(int(linha[0]), int(linha[1]), int(linha[2]))
    
        for i in range(self.n):
            self.visitado.append(0)
            self.pai.append(0)
        
                


    def adiciona_aresta(self, s, t, w):
        self.lista[s].append(Edge(s,t,w))       #dicionario na posicao s recebe aresta
        if not self.D:                          #de s para t, com peso w
            self.lista[t].append(Edge(t,s,w))   #se não direcionado, a mesma aresta é criada
                                                #na posicao t

    def get_list():
        return self.lista


    def DFS(self, s, t):
        stack = [s]
        for i in range(len(self.visitado)):
            self.visitado[i] = 0
            self.pai[i] = 0
        
        while len(stack) > 0:
            u = stack.pop()
            
            if u == t:
                caminho = [u]
                while u != s:
                    u = self.pai[u]
                    caminho = [u] + caminho
                    
                return caminho                  #retorna lista de vértices, de s até t
            
            if self.visitado[u] != 1:
                
                self.visitado[u] = 1
                for v in self.lista[u]:
                    stack.append(v.t)           #v é um objeto Edge, o atributo t tem o destino da aresta
                    if self.pai[v.t] == 0:
                        self.pai[v.t] = u

    def get_w(self, s, t):
        for i in self.lista[s]: #self.lista[s] e uma lista com objetos Edge, preciso achar
                                #o peso da aresta s-t
            if i.t == t:
                return i.w
            
    def gargalo(self, caminho): 
        gargalo = 0
        for i in range(len(caminho)-1):
            gargalo = max(gargalo, self.get_w(caminho[i], caminho[i+1]))    #pegar o peso da aresta entre o vertice
                                                                            #atual e o proximo


    def FF(s, t):

        fluxo = 0
        residual = Grafo(self.g)
        caminho = residual.DFS(s,t)
        gargalo = residual.gargalo(caminho)
        for i in range(len(caminho) - 1):
            w = residual.get_w(caminho[i], caminho[i+1])
            for j in residual.lista[caminho[i]]:
                if j.t == caminho[i+1]:
                    j.f += gargalo
                    if j.reversivel:
                        residual.lista[j.s].append(Edge(j.s, j.t, gargalo, True))
                        j.reversivel = False
                    else:
                        for aresta in residual.lista[j.t]:
                            if aresta.t == j.s:
                                aresta.












        
grafo = Grafo('g1.txt')
print(grafo.DFS(1,2))
