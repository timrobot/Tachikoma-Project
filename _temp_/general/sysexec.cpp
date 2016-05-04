#include "ipcdb.h"

using namespace std;
using namespace arma;

int main() {
  // initialize the database
  robot = new Tachikoma();
  screen = new cube(512, 512, 3);
  
  // start the watchdog (for health reasons)
  watchdog::start();

  // start the visual scanner
  visual::start();
}
