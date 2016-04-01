//
// Created by castoryan on 02.04.16.
//

#include <unistd.h>
#include <iostream>
#include "Loopper.h"



namespace QR_SLAM{

    Loopper::Loopper(){

    }

    void Loopper::Run(){
        while(1){
            std::cout<<"in Loopper"<<std::endl;




            usleep(5000);
        }

    }

}