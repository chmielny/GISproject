#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <list>
#include <cstdio>
#include <climits>
#include <utility>
#include <queue>
#include <algorithm>
#include <functional>
#include <chrono>

using namespace std;

// Funkcja bedaca realizacja algorytmu Dijkstry wykorzystujaca kolejke 
// priorytetowa z biblioteki standardowej C++. Jako argumenty przyjmuje:
// - liste sasiedztwa grafu
// - liczbe wierzcholkow
// - poczatkowy wierzcholek rozpatrywanej sciezki
// - koncowy wierzcholek rozpatrywanej sciezki
// Jako wynik zwraca wektor zawierajacy kolejne wierzcholki ze znalezionej
// sciezki.
std::vector<int> dijkstra(		
					list< pair<int, int> > adjacencyList[], 
                    int vertices,
                    int startVertex,
                    int endVertex
			)
{
	// Zmienna do przechowywania ciagu
	// wierzcholkow bedacych wynikowa 
	// sciezka z alg. Dijkstry
	std::vector<int> returnPath;
	
	// Jezeli wierzcholek poczatkowy lub koncowy
	// jest z poza zakresu to nie wykonuj
	// algorytmu
	if(startVertex < 1 || startVertex > vertices ||
		endVertex < 1 || endVertex > vertices)
	{
		std::cout <<"Bad args in Dijkstra algo" << std::endl;
		return returnPath;
	}

	// Pomocniczy wektor do oznaczania 
	// odwiedzonych wierzcholkow
	std::vector<bool> visited(vertices);
	for(int i = 0; i < vertices; ++i)
		visited[i] = false;
	
	// Wektor do przechowywania poprzednika kazdego
	// rozpatrywanego w algorytmie wierzcholka
	vector<int> predecessor(vertices);

	// Utworzenie kolejki priorytetowej do przechowywania
	// przetwarzanych wierzcholkow	
    priority_queue< pair<int, int> , vector <pair<int, int> > , greater<pair<int, int> > > pq;

	// Utworzenie wektora odleglosci oraz zainicializowanie
	// go najwiekszymi mozliwymi wartosciami (INT_MAX) 
    vector<int> dist(vertices, INT_MAX);
 
	// Wstawienie do kolejki priorytetowej punktu poczatkowego
	// z odlegloscia = 0
    pq.push(make_pair(0, startVertex));
    dist[startVertex - 1] = 0;
	predecessor[startVertex - 1] = startVertex;

	// Dzialanie petli dopoki nie oprozni sie kolejka
	// priorytetowa
    while (!pq.empty())
    {
		// Pierwszym elementem w parze jest minimalna
		// odleglosc od wierzcholka poczatkowego, mozna 
		// ja uzyskac z kolejki priorytetowej za pomoca 
		// top().first. Odleglosc musi byc na pierwszym 
		// miejscu w celu poprawnego sortowania kolejki.
		// Drugim elementem w parze jest etykieta wierzchołka.
        int u = pq.top().second;
        pq.pop();
		if(!visited[u - 1])
		{
			// zmienna 'i' jest uzywana do dostepu do wszystkich
			// sasiadujacych wierzcholkow rozpatrywanego wierzcholka
			list< pair<int, int> >::iterator i;
			for (i = adjacencyList[u - 1].begin(); i != adjacencyList[u - 1].end(); ++i)
			{
				// Pobranie etykiety wierzcholka bedacego sasiadem u
				// oraz jego odleglosci do u
				int v = (*i).first;
				int weight = (*i).second;
				// Jezeli jest krotsza sciezka do v przez u
				// to nastepuje relaksacja
				if (dist[v - 1] > dist[u - 1] + weight)
				{
					predecessor[v - 1] = u;
					// nowa odleglosc dla wierzcholka v
					dist[v - 1] = dist[u - 1] + weight;
					pq.push(make_pair(dist[v - 1], v));
				}
			}
		}
		visited[u - 1]=true;
    }
   
	// Uzyskanie wektora bedacego szukana sciezka
	// na podstawie wektora poprzednikow wierzcholkow	
	returnPath.push_back(endVertex);
	int lastVert = predecessor[endVertex - 1];
	returnPath.push_back(lastVert);
	while(lastVert != startVertex)
	{
		lastVert = predecessor[lastVert - 1];
		returnPath.push_back(lastVert);
	}
	std::reverse(returnPath.begin(), returnPath.end());
	
	return returnPath;
}

// Funkcja bedaca realizacja algorytmu Bellmana-Forda (zmodyfikowany
// algorytm Dijkstry) pozwalajaca na znalezienie najkrotszej sciezki
// z uwzglednieniem ujemnych wag krawedzi. Jako argumenty przyjmuje:
// - liste sasiedztwa grafu
// - liczbe wierzcholkow
// - poczatkowy wierzcholek rozpatrywanej sciezki
// - koncowy wierzcholek rozpatrywanej sciezki
// Jako wynik zwraca wektor zawierajacy kolejne wierzcholki ze znalezionej
// sciezki.
std::vector<int> bellmanFord(
                    list< pair<int, int> > adjacencyList[], 
                    int vertices,
                    int startVertex,
                    int endVertex
               )
{
	// Zmienna do przechowywania ciagu
	// wierzcholkow bedacych wynikowa 
	// sciezka z alg. Dijkstry
	std::vector<int> returnPath;
	
	// Jezeli wierzcholek poczatkowy lub koncowy
	// jest z poza zakresu to nie wykonuj
	// algorytmu
	if(startVertex < 1 || startVertex > vertices ||
		endVertex < 1 || endVertex > vertices)
	{
		std::cout <<"Bad args in Bellman - Ford algo" << std::endl;
		return returnPath;
	}

	// Wektor do przechowywania poprzednika kazdego
	// rozpatrywanego w algorytmie wierzcholka
	vector<int> predecessor(vertices);
	predecessor[startVertex - 1] = startVertex;

    // shortestDistances jest wektorem par
    // pair.first -> najmniejsza odleglosc od wierzcholka poczatkowego
    // pair.second -> wierzcholek poprzedzajacy
    vector< pair<int, int> > shortestDistances(vertices + 1);

	// Zmienne pomocnicze w tym iterator
    list< pair<int, int> >::iterator traverse;
    int i, j;
     
    // Inicjalizacja wektora odleglosci wartosciami maksymalnymi
    for (i = 0; i <= vertices; ++i) 
	{
        shortestDistances[i].first = INT_MAX;
        shortestDistances[i].second = -1;
    }
     
    // W przypadku wierzcholka poczatkowego
	// jego odleglosc do siebie samego wynosi 0
    shortestDistances[startVertex].first = 0;
    shortestDistances[startVertex].second = 0;
     
    // Glowne petle algorytmu
    for (i = 1; i <= vertices - 1; ++i) 
	{
        for (j = 1; j <= vertices; ++j) 
	{    
			// Poniższy kod przeszukuje cala liste sasiedzwa
             
            traverse = adjacencyList[j - 1].begin();
             
            while (traverse != adjacencyList[j - 1].end()) 
			{
                if (shortestDistances[j].first == INT_MAX) 
				{
                    // Przypisanie: traverse = traverse->next;
                    ++traverse;
                    continue;
                }
                 
                // Sprawdzenie czy jest potrzebna relaksacja
                if ((*traverse).second + shortestDistances[j].first < 
                                        shortestDistances[(*traverse).first].first) 
				{
                    // Jezeli tak, to zmieniamy zawartosc wektora z odleglosciami
                    shortestDistances[(*traverse).first].first = (*traverse).second
                                        + shortestDistances[j].first;
                    shortestDistances[(*traverse).first].second = j;
					predecessor[(*traverse).first - 1] = j;
                }
                 
                ++traverse;
            }
        }
    }
     
    // Sprawdzanie czy sa cykle ujemne
    for (j = 1; j <= vertices; ++j) {
        traverse = adjacencyList[j - 1].begin();
         
        while (traverse != adjacencyList[j - 1].end()) {
            // Sprawdzenie kolejnych relaksacji
            if ((*traverse).second + shortestDistances[j].first < 
                                        shortestDistances[(*traverse).first].first) {
                // Kolejne relaksacje sa mozliwe - graf zawiera cykl ujemny
				// Zwracamy pusta sciezke
				std::cout <<"Negative cycle in graph" << std::endl;
				return returnPath;
            }
             
            ++traverse;
        }
    }

	// Uzyskanie wektora bedacego szukana sciezka
	// na podstawie wektora poprzednikow wierzcholkow	
	returnPath.push_back(endVertex);
	int lastVert = predecessor[endVertex - 1];
	returnPath.push_back(lastVert);
	while(lastVert != startVertex)
	{
		lastVert = predecessor[lastVert - 1];
		returnPath.push_back(lastVert);
	}
	std::reverse(returnPath.begin(), returnPath.end());
	
	return returnPath;
     
}

// Pomocnicza funkcja wyswietlajaca liste sasiedztwa
void printAdjacency(int vertices, list<pair<int, int>> *table, string str)
{
	cout << "\nThe Adjacency List " << str << endl;
	// Printing Adjacency List
	for (int i = 0; i < vertices; ++i) {
		printf("table[%d] ", i + 1);
		 
		list< pair<int, int> >::iterator itr = table[i].begin();
		 
		while (itr != table[i].end()) {
			printf(" -> %d(%d)", (*itr).first, (*itr).second);
			++itr;
		}
		printf("\n");
	}
}

// Pomocnicza funkcja wyswietlajaca sciezke
void printPath(int vertices, list<int> *table, string str)
{
	cout << "\nThe Path " << str << endl;
	// Printing Adjacency List
	for (int i = 0; i < vertices; ++i) {
		printf("table[%d] ", i + 1);
		 
		list< int >::iterator itr = table[i].begin();
		 
		while (itr != table[i].end()) {
			printf(" -> %d", (*itr));
			++itr;
		}
		printf("\n");
	}
}

// Glowna funkcja programu
// argumenty wywolania:
// - plik z grafem
// - numer wierzcholka poczatkowego
// - numer wierzcholka koncowego
// - zmienna okreslajaca co wyswietlac na standardowe wyjscie
int main(int argc, char** argv)
{
    if (argc != 5) 
	{
		std::cout << "Zla liczba argumentow wywolania." << std::endl;
		std::cout << "Uzycie: ./gisproject plik_grafu wierzcholek_poczatkowy wierzcholek_koncowy plik wynikowy" << std::endl;
		return 1;
	} else
	{
		int start = atoi(argv[2]);
		int end = atoi(argv[3]);
		int vertices = 0;
		string s = argv[1];
		ifstream file(s);
		string line;
		bool begin_reading = false;
		int x,y,z;
		list<pair<int, int>> *table;
		list<int> *both_paths;
		vector<int> path1, path2, final_path1, final_path2;
		bool path_inverted = false;
		int edge_start, edge_end;
		chrono::microseconds total_time{};
		if (file.is_open() == true)
		{
			std::cout << "Uzyskano dostep do pliku!" << std::endl;
			while (getline(file, line))//pomijanie niepotrzebnych danych i znajdowanie ilości wierzchołków
			{
				if (line == "#") break;
				vertices++;
			}
			table = new list<pair<int, int>> [vertices];
			while (file >> x >> y >> z)//wczytywanie danych
			{
				table[x-1].push_back(pair<int, int>(y, z));
			}
			printAdjacency(vertices, table, "before Dijkstra");
			
			auto start_time = std::chrono::high_resolution_clock::now();// rozpoczecie liczenia czasu algorytmu
			
			path1 = dijkstra(table, vertices, start, end);
			edge_start = path1[0];
			for (int i = 1; (size_t)i < path1.size(); i++)//odwracanie krawędzi
			{
				edge_end = path1[i];
				for (list<pair<int, int>>::iterator iter = table[edge_start-1].begin(); iter != table[edge_start-1].end();)
				{
					if ((*iter).first == edge_end)
					{
						table[edge_end - 1].push_back(pair<int, int>(edge_start, -((*iter).second)));
						iter = table[edge_start - 1].erase(iter);
					}
					else
						iter++;
				}
				edge_start = edge_end;
			}
			printAdjacency(vertices, table, "after Dijkstra");
			path2 = bellmanFord(table, vertices, start, end);
			both_paths = new list<int>[vertices];
			for (int i = 0; (size_t)(i + 1) < path1.size(); i++)//wczytywanie ścieżki 1
			{
				both_paths[path1[i] - 1].push_back(path1[i + 1]);
			}
			printPath(vertices, both_paths, "pierwsza sciezka");
			bool erased;
			for (int i = 0; (size_t)(i + 1) < path2.size(); i++)//wczytywanie ścieżki 2 i kasowanie znoszących się krawędzi
			{
				erased = false;
				for (list<int>::iterator iter = both_paths[path2[i + 1] - 1].begin(); iter != both_paths[path2[i + 1] - 1].end(); iter++)
				{
					if ((*iter) == path2[i])
					{
						both_paths[path2[i + 1] - 1].erase(iter);
						erased = true;
						break;
					}
				}
				if (erased == false)
					both_paths[path2[i] - 1].push_back(path2[i + 1]);
			}
			printPath(vertices, both_paths, "dwie sciezki");
			int to_delete;
			int final_path1_start = start;
			int final_path1_end = start;
			final_path1.push_back(start);
			while (final_path1_end != end)// ekstrakcja sciezki 1
			{
				final_path1.push_back(*(both_paths[final_path1_end - 1].begin()));
				to_delete = final_path1_end;
				final_path1_end = *(both_paths[final_path1_end - 1].begin());
				both_paths[to_delete - 1].erase(both_paths[to_delete - 1].begin());
			}
			int final_path2_start = start;
			int final_path2_end = start;
			final_path2.push_back(start);
			while (final_path2_end != end)// ekstrakcja sciezki 2
			{
				final_path2.push_back(*(both_paths[final_path2_end - 1].begin()));
				to_delete = final_path2_end;
				final_path2_end = *(both_paths[final_path2_end - 1].begin());
				both_paths[to_delete - 1].erase(both_paths[to_delete - 1].begin());
			}

			// zakonczenie liczenia czasu algorytmu
			total_time += chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - start_time);

			for (vector<int>::const_iterator iter = final_path1.begin(); iter != final_path1.end(); iter++)
				cout << *iter << ' ';
			cout << endl;
			for (vector<int>::const_iterator iter = final_path2.begin(); iter != final_path2.end(); iter++)
				cout << *iter << ' ';
			cout << endl;

			std::cout << total_time.count() << std::endl;
		}
		else std::cout << "Dostep do pliku zostal zabroniony!" << std::endl;
		cin.get();
		return 0;
	}
}

