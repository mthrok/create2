#include "create2_driver/create2_driver.hpp"

int main() {
  Create2 c2;
  c2.init();
  c2.start();
  c2.test();
  //c2.stop();
  return 0;
}
