


def reconstruct_path(prev, source_vertex, dest_vertex):
  '''
  Given a populated 'prev' array, a source vertex_index, and destination vertex_index,
  allocate and return an integer array populated with the path from source to destination.
  The first entry of your path should be source_vertex and the last entry should be the dest_vertex.
  If there is no path between source_vertex and dest_vertex, as indicated by hitting a '-1' on the
  path from dest to source, return an empty list.
  '''
  final_path = []
  i = dest_vertex
  final_path.insert(0,prev[dest_vertex]+1)
  c = 1
  while (i <= dest_vertex+1,i > source_vertex):
            if not prev:
                print("No path found")
                return []
            if(prev[i] == source_vertex):
                final_path.append(source_vertex)
                final_path.reverse()
                print(final_path)
                return final_path
            elif (prev[i] != -1):
                    final_path.insert(c, prev[i]);
                    c = c+1
                    if(prev[i]!=-1 or prev[i] == source_vertex):                        
                            i = prev[i]
                    else:
                        print("No path found")
                        return []                                
  print(final_path)
  return final_path


prev1 = [1,2,-1,0,-1,-1,3,6,7]
d = 8
s = 2
reconstruct_path(prev1, s, d)

