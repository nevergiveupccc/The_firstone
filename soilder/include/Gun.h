#pragma once
#include <string>
#include <iostream>
using namespace std;

class Gun
{
public:
    Gun(string type){
        this->_bullet_count = 0;
        this->_type = type;
    }

    void addBullet(int bullet_num);
    bool shoot();

private:
    int _bullet_count;
    string _type;
};