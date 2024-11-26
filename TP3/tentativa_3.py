'''class Edge:
    def __init__(self, s, t, w, r = False): #source, target, weight, reverse
        self.s = s  #int
        self.t = t  #int
        self.w = w  #int
        self.reversivel = True  #Esse atributo será falso se já foi criada uma aresta reversa
                                #a esta no grafo residual
        self.r = r  #bool
        self.f = 0  #fluxo
'''

g1 = "g1.txt"



class Grafo:
    def __init__(self, g, D = True):   #string com path do arquivo, direcionado
        self.n = 0
        self.lista = {}
        self.D = D
        self.g = g
        self.visitado = []  #vetor auxiliar para DFS
        self.pai = []       #vetor auxiliar para recuperar o caminho na DFS
        
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
        self.lista[s].append([t,w,0])           #dicionario na posicao s recebe aresta
        if not self.D:                          #de s para t, com peso w
            self.lista[t].append([s,w,0])       #se não direcionado, a mesma aresta é criada
                                                #na posicao t

    def get_list():
        return self.lista


    def DFS(self, s, t):
        stack = [s]
        for i in range(len(self.visitado)):     #re iniciar listas auxiliares
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
                    stack.append(v[0])           #v[0] tem a informacao de alvo da aresta v
                    if self.pai[v[0]] == 0:
                        self.pai[v[0]] = u

    def get_w(self, s, t):
        for i in self.lista[s]: 
            if i[0] == t:
                return i[1]

    def get_index(self, s, t):
        for i in range(len(self.lista[s])):
            if self.lista[s][i][0] == t:
                return i

            
    def gargalo(self, caminho): 
        gargalo = 0
        for i in range(len(caminho)-1):
            gargalo = max(gargalo, self.get_w(caminho[i], caminho[i+1]))    #pegar o peso da aresta entre o vertice
                                                                            #atual e o proximo
        return gargalo

    def augument(self, caminho, b):
        for i in range(len(caminho) - 1):
            self.lista[caminho[i]][self.get_index(caminho[i],caminho[i+1])][2] += b

    def lidar_residual(self, caminho, b):
        for i in range(len(caminho) - 1):
            self.lista[caminho[i]][self.get_index(caminho[i],caminho[i+1])][1] -= b
            aux = self.get_index(caminho[i+1],caminho[i])
            if aux == None:     #se o loop do metodo get_index nao retornar, a aresta nao existe e preciso criar ela
                self.lista[i+1].append([i, b, 0])
            else:               #criei a variavel aux pra nao precisar fazer o mesmo loop duas vezes
                self.lista[caminho[i+1]][aux][1] += b



    def FF(self, s, t):
        for i in self.lista:
            for j in self.lista[i]:
                j[2] = 0        #zerar os fluxos
        fluxo = 0
        residual = Grafo(g1)
        caminho = [t]
        while caminho[-1] == t:
            caminho = residual.DFS(s,t)
            gargalo = residual.gargalo(caminho)

            
            self.augument(caminho, gargalo)         #aumentar fluxo nas arestas do grafo original
            residual.lidar_residual(caminho, gargalo)     #diminuir a capacidade nas arestas originais do grafo residual
            fluxo += gargalo                              

        return fluxo










        
grafo = Grafo(g1)
print(grafo.FF(1,2))
