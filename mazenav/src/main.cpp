#include <thread>
#include <fusion.h>
#include <globalnav.h>
#include <localnav.h>

using namespace std;

int main()
{
    thread fusionT = thread(mainFusion);
    thread globNavT = thread(mainGlobalNav);
    thread locNavT = thread(mainLocalNav);

    fusionT.join();
    globNavT.join();
    locNavT.join();

    return 0;
}