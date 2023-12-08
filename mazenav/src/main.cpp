#include <thread>
#include <fusion.h>
#include <globalnav.h>
#include <localnav.h>

using namespace std;

int main()
{
    thread fusionT = thread(mainFusion);
    thread globNavT = thread(mainFusion);
    thread locNavT = thread(mainFusion);

    fusionT.join();
    globNavT.join();
    locNavT.join();

    return 0;
}