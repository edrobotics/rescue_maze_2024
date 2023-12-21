#include <thread>
#include <fusion/fusion.h>
#include <globalNav/globalNav.h>
#include <localNav/localNav.h>

using namespace std;

int main()
{
    fusion::main();
    thread fusionT = thread(fusion::main);
    thread globNavT = thread(globalNav::main);
    thread locNavT = thread(localNav::main);

    fusionT.join();
    globNavT.join();
    locNavT.join();

    return 0;
}