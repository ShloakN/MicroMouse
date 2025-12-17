#ifndef COMPASSH
#define COMPASSH

class Compass{

private:
  char globalOrient = 'B';//Start Bottom faceing
  int rows;
  int cols;
  
public:

  int coordinate[2];

  Compass(int row, int col){
    rows = row;
    cols = col;
  }
  
  void setOrient(char dir){
    globalOrient = dir;
  }

  char getOrient(){
    return globalOrient;
  }

  bool setCoordinate(int r, int c) {
        if ((r >= 0 && r < rows) && (c >= 0 && c < cols)) {
            coordinate[0] = r;
            coordinate[1] = c;
            return true;
        }
        return false;
  }

  void show(){
    Serial.print(globalOrient);
    Serial.print(", (");
    Serial.print(coordinate[0]);
    Serial.print(", ");
    Serial.print(coordinate[1]);
    Serial.println(")");
  }

  void updateOrient(char newDir) {

    switch (newDir) {

      case 'R':
        switch (globalOrient) {
          case 'T':
            globalOrient = 'R';
            break;
          case 'R':
            globalOrient = 'B';
            break;
          case 'B':
            globalOrient = 'L';
            break;
          case 'L':
            globalOrient = 'T';
            break;
          default:
            break;
        }
        break;
      case 'L':
        switch (globalOrient) {
          case 'T':
            globalOrient = 'L';
            break;
          case 'R':
            globalOrient = 'T';
            break;
          case 'B':
            globalOrient = 'R';
            break;
          case 'L':
            globalOrient = 'B';
            break;
          default:
            break;
        }
        break;
      default:
        break;
    }
  }

  void updateCoord(){
        /*
          Assumes that robot always moves forward
        */

        switch (globalOrient) {
            case 'T':  
                setCoordinate(coordinate[0]-1, coordinate[1]);
                break;
            case 'R': 
                setCoordinate(coordinate[0], coordinate[1]+1);
                break;
            case 'L':  
                setCoordinate(coordinate[0], coordinate[1]-1);
                break;
            case 'B':  
                setCoordinate(coordinate[0]+1, coordinate[1]);
                break;
            default:
                return;
        }

  }

};


#endif