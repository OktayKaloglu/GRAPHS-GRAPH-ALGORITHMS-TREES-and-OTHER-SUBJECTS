using System;
using System.Collections.Generic;
using System.Linq;

namespace Project4
{
    class Karakter
    {
        public int frekans = 0;
        public String karakter;
        public string code;
        public Karakter(String car)
        {
            this.frekans = 1;
            this.karakter = car.ToString();
        }

        public Karakter(String car, int tekrar)
        {
            this.frekans = tekrar;
            this.karakter = car.ToString();
        }

        public string ToString()
        {
            return karakter + "     " + frekans + "     " + code;
        }


    }
    class HuffmanEncodingTree
    {
        private Node root;
        private List<Node> öncelikKuyruğu;
        private Karakter[] Karakterler;

        internal class Node
        {
            internal Karakter karak;

            internal Node leftChild;
            internal Node rightChild;

            public Node(Karakter kar)
            {
                this.karak = kar;
                this.leftChild = null;
                this.rightChild = null;


            }
        }
        public HuffmanEncodingTree(Karakter[] karakterler)//çok tekrardan az tekrara doğru sıralı karakterler neslelerini içeren dizi girişi yapıldığı kabul edilmiştir.
        {
            Karakterler = karakterler;
            öncelikKuyruğu = new List<Node>();
            for (int i = 0; i < karakterler.Length; i++)
            {
                Node node = new Node(karakterler[i]);
                öncelikKuyruğu.Add(node);
            }
            makeRoot();

        }

        private void makeRoot()
        {
            int nodeSayısı = öncelikKuyruğu.Count;
            for (int i = nodeSayısı - 1; i > 0; i--)//öncelik kuğruğunda 1 tane item kalana kadar tekrarlanmalı. kkalan item ise bizim kökümüz olmuştur.
            {
                Node son = öncelikKuyruğu[i];
                Node sondanÖnceki = öncelikKuyruğu[i - 1];

                öncelikKuyruğu.RemoveAt(i);//son iki itemın listeden çıkartılması
                öncelikKuyruğu.RemoveAt(i - 1);


                Karakter geriEklenecek = new Karakter(sondanÖnceki.karak.karakter + son.karak.karakter, son.karak.frekans + sondanÖnceki.karak.frekans);
                Node geriEkelenecekNode = new Node(geriEklenecek);
                geriEkelenecekNode.rightChild = son;
                geriEkelenecekNode.leftChild = sondanÖnceki;
                bool durum = true;
                for (int y = 0; y < öncelikKuyruğu.Count; y++)
                {
                    if (öncelikKuyruğu[y].karak.frekans < geriEkelenecekNode.karak.frekans)
                    {
                        öncelikKuyruğu.Insert(y, geriEkelenecekNode);
                        durum = false;
                        break;
                    }
                }
                if (durum)
                {
                    öncelikKuyruğu.Add(geriEkelenecekNode);
                }

            }
            root = öncelikKuyruğu[0];
        }
        private string findCharCode(string karakter)
        {//verilen harfin ağaçtan faydalanarak kodunun üretilmesi
            Node tempNode = root;
            string code = "";
            while (tempNode != null && tempNode.leftChild != null)//node boş ise tek karaktere kadar inilmiştir.
            {
                if (tempNode.leftChild.karak.karakter.Contains(karakter))
                {
                    tempNode = tempNode.leftChild;
                    code += "0";
                }
                else
                {
                    tempNode = tempNode.rightChild;
                    code += "1";
                }
            }
            return code;


        }
        public void harflerinCodeları()
        {
            Console.WriteLine("harf" + " " + "frekans" + " " + "code");
            for (int i = 0; i < Karakterler.Length; i++)
            {
                Karakterler[i].code = findCharCode(Karakterler[i].karakter);
                Console.WriteLine(Karakterler[i].ToString()); ;
            }
        }
        public string coder(string cümle)//cümle girilecek "asd ss ad". "010101001111" gibi string döndürücek
        {
            string code = "";


            for (int i = 0; i < cümle.Length; i++)
            {
                code += findCharCode(cümle.Substring(i, 1));

            }
            return code;
        }
        public string decoder(string code)//code girilecek "001101010" gibi 
        {
            String cümle = "";
            int temp = 0;//her zaman başlanacak yeri gösterir
            Node tempNode = root;
            while (temp < code.Length)
            {
                while (tempNode.leftChild != null)//
                {
                    if (code[temp] != '0')
                    { //soldaki node değilse
                        temp++;
                        tempNode = tempNode.rightChild;
                    }
                    else//0 gelmiştir demmeki  sol çocuk aranılan harfi içerir.
                    {
                        temp++;
                        tempNode = tempNode.leftChild;

                    }
                }
                cümle += tempNode.karak.karakter;
                tempNode = root;//kodun devamı için tempnodun en başı göstermesi gerekir.

            }
            return cümle;


        }

    }


    class Dijkstra
    {
        // A utility function to find the 
        // vertex with minimum distance 
        // value, from the set of vertices 
        // not yet included in shortest 
        // path tree 
        static int V = 8;
        int minDistance(int[] dist,
                        bool[] sptSet)
        {
            // Initialize min value 
            int min = int.MaxValue, min_index = -1;

            for (int v = 0; v < V; v++)
                if (sptSet[v] == false && dist[v] <= min)
                {
                    min = dist[v];
                    min_index = v;
                }

            return min_index;
        }

        // A utility function to print 
        // the constructed distance array 
        void printSolution(int[] dist)
        {
            Console.Write("Vertex \t\t Distance "
                          + "from Source\n");
            for (int i = 0; i < V; i++)
                Console.Write(i + " \t\t " + dist[i] + "\n");
        }

        // Funtion that implements Dijkstra's 
        // single source shortest path algorithm 
        // for a graph represented using adjacency 
        // matrix representation 
        public void dijkstra(int[,] graph, int src)
        {
            V = graph.GetLength(0);
            int[] dist = new int[V]; // The output array. dist[i] 
                                     // will hold the shortest 
                                     // distance from src to i 
            
            // sptSet[i] will true if vertex 
            // i is included in shortest path 
            // tree or shortest distance from 
            // src to i is finalized 
            bool[] sptSet = new bool[V];

            // Initialize all distances as 
            // INFINITE and stpSet[] as false 
            for (int i = 0; i < V; i++)
            {
                dist[i] = int.MaxValue;
                sptSet[i] = false;
            }

            // Distance of source vertex 
            // from itself is always 0 
            dist[src] = 0;

            // Find shortest path for all vertices 
            for (int count = 0; count < V - 1; count++)
            {
                // Pick the minimum distance vertex 
                // from the set of vertices not yet 
                // processed. u is always equal to 
                // src in first iteration. 
                int u = minDistance(dist, sptSet);

                // Mark the picked vertex as processed 
                sptSet[u] = true;

                // Update dist value of the adjacent 
                // vertices of the picked vertex. 
                for (int v = 0; v < V; v++)

                    // Update dist[v] only if is not in 
                    // sptSet, there is an edge from u 
                    // to v, and total weight of path 
                    // from src to v through u is smaller 
                    // than current value of dist[v] 
                    if (!sptSet[v] && graph[u, v] != 0 && dist[u] != int.MaxValue && dist[u] + graph[u, v] < dist[v])
                        dist[v] = dist[u] + graph[u, v];
            }

            // print the constructed distance array 
            printSolution(dist);
        }
    }


    class MST//Prim’s MST 
    {

        // Number of vertices in the graph 
        private int V = 9;

        // A utility function to find 
        // the vertex with minimum key 
        // value, from the set of vertices 
        // not yet included in MST
        public MST(int[,] graph)
        {
            this.V = graph.GetLength(0);
            primMST(graph);
        }
        private int minKey(int[] key, bool[] mstSet)
        {

            // Initialize min value 
            int min = int.MaxValue, min_index = -1;

            for (int v = 0; v < V; v++)
                if (mstSet[v] == false && key[v] < min)
                {
                    min = key[v];
                    min_index = v;
                }

            return min_index;
        }

        // A utility function to print 
        // the constructed MST stored in 
        // parent[] 
        private void printMST(int[] parent, int[,] graph)
        {
            Console.WriteLine("Edge" + " \t\t " + "Weight");
            for (int i = 1; i < V; i++)
                Console.WriteLine(parent[i] + " - " + i +" \t\t "+ graph[i, parent[i]]);
        }

        // Function to construct and 
        // print MST for a graph represented 
        // using adjacency matrix representation 
        public  void primMST(int[,] graph)
        {

            // Array to store constructed MST 
            int[] parent = new int[V];

            // Key values used to pick 
            // minimum weight edge in cut 
            int[] key = new int[V];

            // To represent set of vertices 
            // included in MST 
            bool[] mstSet = new bool[V];

            // Initialize all keys 
            // as INFINITE 
            for (int i = 0; i < V; i++)
            {
                key[i] = int.MaxValue;
                mstSet[i] = false;
            }

            // Always include first 1st vertex in MST. 
            // Make key 0 so that this vertex is 
            // picked as first vertex 
            // First node is always root of MST 
            key[0] = 0;
            parent[0] = -1;

            // The MST will have V vertices 
            for (int count = 0; count < V - 1; count++)
            {

                // Pick thd minimum key vertex 
                // from the set of vertices 
                // not yet included in MST 
                int u = minKey(key, mstSet);

                // Add the picked vertex 
                // to the MST Set 
                mstSet[u] = true;

                // Update key value and parent 
                // index of the adjacent vertices 
                // of the picked vertex. Consider 
                // only those vertices which are 
                // not yet included in MST 
                for (int v = 0; v < V; v++)

                    // graph[u][v] is non zero only 
                    // for adjacent vertices of m 
                    // mstSet[v] is false for vertices 
                    // not yet included in MST Update 
                    // the key only if graph[u][v] is 
                    // smaller than key[v] 
                    if (graph[u, v] != 0 && mstSet[v] == false
                        && graph[u, v] < key[v])
                    {
                        parent[v] = u;
                        key[v] = graph[u, v];
                    }
            }

            // print the constructed MST 
            printMST(parent, graph);
        }
    }

    class BFT
    {

        // No. of vertices     
        private int _V;

        //Adjacency Lists 
        LinkedList<int>[] _adj;

        public BFT(int V)
        {
            _adj = new LinkedList<int>[V];
            for (int i = 0; i < _adj.Length; i++)
            {
                _adj[i] = new LinkedList<int>();
            }
            _V = V;
        }

        // Function to add an edge into the graph 
        public void AddEdge(int v, int w)
        {
            _adj[v].AddLast(w);

        }

        // Prints BFS traversal from a given source s 
        public void BFS(int s)
        {

            // Mark all the vertices as not
            // visited(By default set as false) 
            bool[] visited = new bool[_V];
            for (int i = 0; i < _V; i++)
                visited[i] = false;

            // Create a queue for BFS 
            LinkedList<int> queue = new LinkedList<int>();

            // Mark the current node as 
            // visited and enqueue it 
            visited[s] = true;
            queue.AddLast(s);

            while (queue.Any())
            {

                // Dequeue a vertex from queue 
                // and print it
                s = queue.First();
                Console.Write(s + " ");
                queue.RemoveFirst();

                // Get all adjacent vertices of the 
                // dequeued vertex s. If a adjacent
                // has not been visited, then mark it 
                // visited and enqueue it 
                LinkedList<int> list = _adj[s];

                foreach (var val in list)
                {
                    if (!visited[val])
                    {
                        visited[val] = true;
                        queue.AddLast(val);
                    }
                }
            }
        }
    }


        class Program
    {
        static void for2()
        {
            //HuffmanEncodingTree için karakterler ve frekansları ayrı girilebilir
            //veya belirli bir metin ile giriş yapılabilir bunu için metinin icelenip karakterlerin frekanslarına göre bir karakterler dizisi oluşturulmalıdır.
            //bu methotta sadece harflerin ve frekansların verileceğini varsaydım
            char[] car = { ' ', 'a', 'b', 'c', 'd', 'e', 'f', 'g' };
            int[] tekrar = { 55, 50, 45, 40, 35, 30, 25, 20 };
            Karakter[] karakterler = new Karakter[car.Length];
            for (int i = 0; i < car.Length; i++)
            {
                Karakter kar = new Karakter(car[i].ToString(), tekrar[i]);
                karakterler[i] = kar;
            }

            HuffmanEncodingTree ağac = new HuffmanEncodingTree(karakterler);
            ağac.harflerinCodeları();
            Console.WriteLine("abc ab abb : " + ağac.coder("abc ab abb".ToLower()));
            Console.WriteLine("0000010111100000111000001001 çözülmüşü : " + ağac.decoder("0000010111100000111000001001"));

        }




        static void Main(string[] args)
        {
            Console.BackgroundColor = ConsoleColor.White;
            Console.Clear();
            Console.ForegroundColor = ConsoleColor.Black;

            for2();
            //4
                                         //a,b,c,d,e,f,g,h,j,   
            int[,] graph = new int[,] { { 0, 4, 0, 0, 0, 0, 0, 8, 0 },//a
                                      { 4, 0, 8, 0, 0, 0, 0, 11, 0 },//b
                                      { 0, 8, 0, 7, 0, 4, 0, 0, 2 },//c
                                      { 0, 0, 7, 0, 9, 14, 0, 0, 0 },//d
                                      { 0, 0, 0, 9, 0, 10, 0, 0, 0 },//e
                                      { 0, 0, 4, 14, 10, 0, 2, 0, 0 },//f
                                      { 0, 0, 0, 0, 0, 2, 0, 1, 6 },//g
                                      { 8, 11, 0, 0, 0, 0, 1, 0, 7 },//h
                                      { 0, 0, 2, 0, 0, 0, 6, 7, 0 } };//j

            //4-a
            Console.WriteLine("Dijkstra’s Shortest Path ");
            Dijkstra dij = new Dijkstra();
            dij.dijkstra(graph,0);
            Console.WriteLine();
            Console.WriteLine();
            Console.WriteLine();

            //4-b
            Console.WriteLine("Prim’s MST ");
            MST mst = new MST(graph);
            Console.WriteLine();
            Console.WriteLine();
            Console.WriteLine();


            //4-b
            Console.WriteLine(" BFT (Breadth-First Traverse) ");
            BFT bft = new BFT(4);
            bft.AddEdge(0, 1);
            bft.AddEdge(0, 2);
            bft.AddEdge(1, 2);
            bft.AddEdge(2, 0);
            bft.AddEdge(2, 3);
            bft.AddEdge(3, 3);
            Console.WriteLine("Following is Breadth First " + "Traversal(starting from vertex 2)");
            bft.BFS(2);
            Console.WriteLine();
            Console.WriteLine();
            Console.WriteLine();
        }
    }
}

