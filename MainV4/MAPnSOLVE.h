#ifndef MAPNSOLVE
#define MAPNSOLVE
#include "Queue.h"


//int end[4][2] = {{7,7},{7,8},{8,7},{8,8}};
int end[1][2]    = {{2,1}};
int start[2]  = {0,0};

class BinMaze{

  public:
    int maze[4][3];
    int pathMatrix[4][3]; // contains all visited locations, updated with every cell update
    char pathMap[150];
    int rows = 4;
    int cols = 3;
    int walls[4] = {1, 2, 4, 8};//T R B L

  BinMaze(int fill = 1){
    init(fill);
    for(int i = 0; i<rows; i++){
      for(int j = 0; j<cols; j++){
        pathMatrix[i][j] = 0;
      }
    }
  }

  void init(int fill){
    int fillVal = 15;
    if(fill == 0){
      fillVal = 0;
    }

    for(int i = 0; i<rows; i++){
      for(int j = 0; j<cols; j++){
        maze[i][j] = fillVal;
      }
    }
  }

  void updateMaze(char dir, int r, int c, bool bot_F, bool bot_R, bool bot_B, bool bot_L){

    switch (dir) {
      case 'T':
        maze[r][c] = 1*bot_F + 2*bot_R + 4*bot_B + 8*bot_L ;
        break;
      case 'R':
        maze[r][c] = 1*bot_L + 2*bot_F + 4*bot_R + 8*bot_B ;
        break;
      case 'B':
        maze[r][c] = 1*bot_B + 2*bot_L + 4*bot_F + 8*bot_R ;
        break;
      case 'L':
        maze[r][c] = 1*bot_R + 2*bot_B + 4*bot_L + 8*bot_F ;
        break;
      default:
        return;
    }
    pathMatrix[r][c] = 9;
  }

  void maze2PathMatrix(){
    pathMatrix[start[0]][start[1]] = 0;
    
    for (int i=0; i<rows; i++) {
      for (int j=0; j<cols; j++) {

        if (maze[i][j]==15) {
          continue;
        }

        if ((~maze[i][j] & walls[0]) == walls[0] && isValid(i-1, j)) {
          pathMatrix[i-1][j] = 0;
        }
        if ((~maze[i][j] & walls[1]) == walls[1] && isValid(i, j+1)) {
          pathMatrix[i][j+1] = 0;
        }
        if ((~maze[i][j] & walls[2]) == walls[2] && isValid(i+1, j)) {
          pathMatrix[i+1][j] = 0;
        }
        if ((~maze[i][j] & walls[3]) == walls[3] && isValid(i, j-1)) {
          pathMatrix[i][j-1] = 0;
        }
      }
    }
  }

  void path2PathMatrix(int startr, int startc){
    
    int x = startr, y = startc;
    pathMatrix[x][y] = 9;
    for (char dir : pathMap) {

      if (dir == 'X')
          continue;
      else if (dir == 'T')
          x -= 1;
      else if (dir == 'R')
          y += 1;
      else if (dir == 'L')
          y -= 1;
      else if (dir == 'B')
          x += 1;
      else
          break;
      
      pathMatrix[x][y] = 9;
    }
    
  }

  bool checkWall(int r, int c, char dir){

    switch (dir) {
     case 'T':
      return (maze[r][c] & walls[0]) && (isValid(r-1, c));
     case 'R':
      return (maze[r][c] & walls[1]) && (isValid(r, c+1));
     case 'B':
      return (maze[r][c] & walls[2]) && (isValid(r+1, c));
     case 'L':
      return (maze[r][c] & walls[3]) && (isValid(r, c-1));
     default:
      return false;
    }
  }

  bool isValid(int r, int c){

    return(r>=0 && r<rows && c>=0 && c< cols);
  }

  void clearPathMap(){
    for(int i = 0; i < 150; i++){
      pathMap[i] = '\0';
    }
  }

  void display(){
    for (int i=0; i<rows; i++) {
      for (int j=0; j<cols; j++) {
        Serial.print(maze[i][j], BIN);
        Serial.print(" ");
      }
      Serial.println();
    }
  }

  void plotPathMatrix(){
    for(int i = 0; i<rows; i++){
      for(int j = 0; j<cols; j++){
        Serial.print(pathMatrix[i][j]);
        Serial.print(" ");
      }
      Serial.println();
    }
  }

  void showPath(){
    for (char ele : pathMap){
      Serial.print(ele);
      Serial.print(", ");
    }
    Serial.println();
  }

};

class FloodFill{

  public:
    BinMaze& maze;
    int rows;
    int cols;
    bool visited[4][3];

  FloodFill(BinMaze& maze1): maze(maze1){

    rows = maze.rows;
    cols = maze.cols;
    init(true);
  }

  void init(bool markStart = true){
    for (int i = 0; i<rows; i++) {
      for (int j =0; j<cols; j++) {
        visited[i][j] = false;
      }
    }
    visited[start[0]][start[1]] = markStart;

  }

  void compute(){
    
    Queue queue;

    //if(queue.isEmpty())
    //  Serial.println("Empty Queue");

    queue.enQueue(start[0], start[1]);

    while (!queue.isEmpty()) {

      Queue::Node* node = queue.deQueue();
      
      if((node->row == end[0][0] && node->col == end[0][1])/* || (node->row == end[1][0] && node->col == end[1][1]) ||
         (node->row == end[2][0] && node->col == end[2][1]) || (node->row == end[3][0] && node->col == end[3][1])*/){
          
          int len = 0;
          for(char element : node->path)
          {
            maze.pathMap[len] = element;
            Serial.print(element);
            Serial.print(" "); 
            len++;
          }
          maze.pathMap[len] = 'Z';
          Serial.println();
          Serial.print("length = ");
          Serial.println(len-1);

          queue.clearQueue();
          queue.freeNode(node);
          return;
        }

        // check bottom
      if(maze.isValid(node->row+1, node->col) && !visited[node->row+1][node->col]){

          if(!maze.checkWall(node->row, node->col, 'B')){
            visited[node->row+1][node->col] = true;
            queue.enQueue(node->row+1, node->col, node->path, 'B');
          }
      }
        // check top
      if(maze.isValid(node->row-1, node->col) && !visited[node->row-1][node->col]){

          if(!maze.checkWall(node->row, node->col, 'T')){
            visited[node->row-1][node->col] = true;
            queue.enQueue(node->row-1, node->col, node->path, 'T');
          }
      }
        // check right
      if(maze.isValid(node->row, node->col+1) && !visited[node->row][node->col+1]){

          if(!maze.checkWall(node->row, node->col, 'R')){
            visited[node->row][node->col+1] = true;
            queue.enQueue(node->row, node->col+1, node->path, 'R');
          }
      }
        // check left
      if(maze.isValid(node->row, node->col-1) && !visited[node->row][node->col-1]){

          if(!maze.checkWall(node->row, node->col, 'L')){
            visited[node->row][node->col-1] = true;
            queue.enQueue(node->row, node->col-1, node->path, 'L');
          }
      }

      queue.freeNode(node);
    }

    queue.clearQueue();
    Serial.println("No path found");
  }

  // For Finding possible approach path.
  void computeStart2End(int startr,int startc){

    Queue queue;

    queue.enQueue(startr, startc);
    //maze.clearPathMap();

    while (!queue.isEmpty()) {

      Queue::Node* node = queue.deQueue();
      
      if((node->row == end[0][0] && node->col == end[0][1])/* || (node->row == end[1][0] && node->col == end[1][1]) ||
         (node->row == end[2][0] && node->col == end[2][1]) || (node->row == end[3][0] && node->col == end[3][1])*/){
          
          int len = 0;
          Serial.print("S2E: ");

          for(char element : node->path)
          {
            maze.pathMap[len] = element;
            Serial.print(maze.pathMap[len]);
            Serial.print(" "); 
            len++;            
          }
          maze.pathMap[len] = 'Z';
          Serial.println();
          Serial.print("length = ");
          Serial.println(len-1); 
          maze.plotPathMatrix();

          queue.clearQueue();
          queue.freeNode(node);
          return;
        }

        // check bottom
      if(maze.isValid(node->row+1, node->col) && !visited[node->row+1][node->col]){

          if(!maze.checkWall(node->row, node->col, 'B')){
            visited[node->row+1][node->col] = true;
            queue.enQueue(node->row+1, node->col, node->path, 'B');
          }
      }
        // check top
      if(maze.isValid(node->row-1, node->col) && !visited[node->row-1][node->col]){

          if(!maze.checkWall(node->row, node->col, 'T')){
            visited[node->row-1][node->col] = true;
            queue.enQueue(node->row-1, node->col, node->path, 'T');
          }
      }
        // check right
      if(maze.isValid(node->row, node->col+1) && !visited[node->row][node->col+1]){

          if(!maze.checkWall(node->row, node->col, 'R')){
            visited[node->row][node->col+1] = true;
            queue.enQueue(node->row, node->col+1, node->path, 'R');
          }
      }
        // check left
      if(maze.isValid(node->row, node->col-1) && !visited[node->row][node->col-1]){

          if(!maze.checkWall(node->row, node->col, 'L')){
            visited[node->row][node->col-1] = true;
            queue.enQueue(node->row, node->col-1, node->path, 'L');
          }
      }

        queue.freeNode(node);
    }
    queue.clearQueue();
    Serial.println("No path found");
  }


  void computeEnd2Start(int startr,int startc){
    
    init(false);                          // Reset Visited matrix
    maze.plotPathMatrix();              
    maze.clearPathMap();

    PriorityQueue queue;

    queue.enQueue(startr, startc);

    while (!queue.isEmpty()) {

      PriorityQueue::Node* node = queue.deQueue();
      
      if((node->row == start[0] && node->col == start[1])/* || (node->row == end[1][0] && node->col == end[1][1]) ||
         (node->row == end[2][0] && node->col == end[2][1]) || (node->row == end[3][0] && node->col == end[3][1])*/){

          Serial.print("E2S: ");

          int len = 0;
          for(char element : node->path)
          {
            maze.pathMap[len] = element;
            Serial.print(maze.pathMap[len]);
            Serial.print(" "); 
            len++;            
          }
          maze.pathMap[len] = 'Z';
          Serial.println();
          Serial.print("length = ");
          Serial.println(len-1);

          queue.clearQueue();
          queue.freeNode(node);
          return;
        }

        // check bottom
      if(maze.isValid(node->row+1, node->col) && !visited[node->row+1][node->col]){

          int cost = (maze.pathMatrix[node->row+1][node->col] == 9)? 12 : 0;
          if(!maze.checkWall(node->row, node->col, 'B')){
            visited[node->row+1][node->col] = true;
            queue.enQueue(node->row+1, node->col, node->path, 'B', cost);
          }
      }
        // check top
      if(maze.isValid(node->row-1, node->col) && !visited[node->row-1][node->col]){

          int cost = (maze.pathMatrix[node->row-1][node->col] == 9)? 12 : 0;
          if(!maze.checkWall(node->row, node->col, 'T')){
            visited[node->row-1][node->col] = true;
            queue.enQueue(node->row-1, node->col, node->path, 'T', cost);
          }
      }
        // check right
      if(maze.isValid(node->row, node->col+1) && !visited[node->row][node->col+1]){

          int cost = (maze.pathMatrix[node->row][node->col+1] == 9)? 12 : 0;
          if(!maze.checkWall(node->row, node->col, 'R')){
            visited[node->row][node->col+1] = true;
            queue.enQueue(node->row, node->col+1, node->path, 'R', cost);
          }
      }
        // check left
      if(maze.isValid(node->row, node->col-1) && !visited[node->row][node->col-1]){

          int cost = (maze.pathMatrix[node->row][node->col-1] == 9)? 12 : 0;
          if(!maze.checkWall(node->row, node->col, 'L')){
            visited[node->row][node->col-1] = true;
            queue.enQueue(node->row, node->col-1, node->path, 'L', cost);
          }
      }

      queue.freeNode(node);
    }

    queue.clearQueue();
    Serial.println("No Return path found");
  }

};





#endif