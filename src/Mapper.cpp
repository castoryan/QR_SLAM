//
// Created by castoryan on 02.04.16.
//

#include <unistd.h>
#include <iostream>
#include "Mapper.h"


namespace QR_SLAM{

    Mapper::Mapper(){

    }

    void Mapper::Run(){
        while(1){
            std::cout<<"in Mapper"<<std::endl;



            usleep(5000);
        }
    }


}