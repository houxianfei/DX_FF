#include <DX_FF.h>
#include <iostream>

int main(int argc,char** argv) {
  if (argc!=2) return 2;
  FF::ff_driver driver;
  auto loaded=driver.load_training_archive(argv[1]);
  if (!loaded) { std::cerr << loaded.result.message << '\n'; return 1; }
  if (loaded.value.selectedChannel<0 || loaded.value.selectedChannel>2)
    return 1;
  return 0;
}
