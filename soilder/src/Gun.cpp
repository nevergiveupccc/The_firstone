#include "Gun.h"
#include <iostream>

void Gun::addBullet(int bullet_num)
{
    this->_bullet_count += bullet_num;
}

bool Gun::shoot()
{
    if (this->_bullet_count <= 0)
    {
        cout << "枪里没有子弹" << endl;
        return false;
    }

    this->_bullet_count--;
    cout << "shoot successfully!" << endl;
    return true;
    
}