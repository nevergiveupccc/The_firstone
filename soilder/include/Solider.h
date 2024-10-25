#pragma once
#include <string>
#include "Gun.h"
using namespace std;

class Solider
{

public:
    Solider(string name);
    ~Solider();
    void addGun(Gun* ptr_gun);
    void addBulletToGun(int num);
    bool fire();
    
private:
    string _name;
    Gun* _ptr_gun;
};

