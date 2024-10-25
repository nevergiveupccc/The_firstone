#include "Gun.h"
#include "Solider.h"

void test()
{
    
    Solider sanduo("xusanduo");
    // Gun* AK47 = new Gun("AK47");
    // sanduo.addGun(AK47);

    sanduo.addGun(new Gun("AK47"));
    sanduo.addBulletToGun(20);
    sanduo.fire();

}

int main()
{
    cout << "This is a test string ..." << endl;
    test();
    return 0;
}
