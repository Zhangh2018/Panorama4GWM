/***********************************************************************
 *
 * Mario Created on   : 2019-06-11 14:01
 * Filename      : EncoderQueue.h
 * Function      : 读取里程计数据
 *
 *************************************************************************/

# pragma once

#include <deque>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include "utils/Timer.hpp"


// 里程计的数据结构
struct Encoder{
    int x;
    int y;
    int theta;
};

class EncoderQueue
{
public:

	EncoderQueue(const int queSize = 30):queSize(queSize)
	{

        char fileName[255];
        FILE* fp = fopen("encoderconfig.txt", "r");
	    if (fp == NULL)
	    {
		    printf("could not found config.txt\n");
	    }

	    fscanf(fp, "encoderpath: %s\n", fileName);
	    fclose(fp);
        printf("encoderpath = %s\n", fileName);

        encoderReader = fopen(fileName,"r");

		// 去除第一行
		char tmpbuf[1024];
		fgets(tmpbuf, sizeof(tmpbuf), encoderReader);

        counter = 0;
		daemon = std::move(std::thread(&EncoderQueue::prefetch, this));
	}

	void prefetch()
	{
        while(1){

		std::unique_lock<std::mutex> lock{ encoderQueueMutex };

		int n = queSize - encoderQueue.size();

		while (n-- > 0){

            ++ counter;

            Encoder singleEncoder;

		    int tmpTime1;
		    int tmpTime2;
		    int tmpTime3;
		    int tmpGPS1;
		    int tmpGPS2;
		    int tmpGPS3;
            int tmpStatus;

		    if(fscanf(encoderReader,"%d-%d:%d %d %d %d %d %d %d %d\n",
			    &tmpTime1,
			    &tmpTime2,
			    &tmpTime3,
                &tmpStatus,
			    &singleEncoder.x,
			    &singleEncoder.y,
			    &singleEncoder.theta,
			    &tmpGPS1,
			    &tmpGPS2,
			    &tmpGPS3)<0)
                return;

            if(singleEncoder.x < 0)
                singleEncoder.x = singleEncoder.x + 2*32768;

            printf("+++++++++++++++++Debug: frameCounter= %d,  tmpTime1 = %d, tmpTime2 = %d, tmpTime3 = %d\n", counter, tmpTime1, tmpTime2, tmpTime3);
            printf("+++++++++++++++++Debug: frameCounter= %d,  tmpTime1 = %d, tmpStatus = %d, tmpGPS1 = %d\n", counter, tmpTime1, tmpStatus, tmpGPS1);
            printf("+++++++++++++++++Debug: frameCounter= %d,  x = %d, y = %d, z = %d\n", counter,singleEncoder.x, singleEncoder.y, singleEncoder.theta);

			encoderQueue.push_back(singleEncoder);

		}
		popVar.notify_one();
		fetchVar.wait_for(lock, std::chrono::milliseconds(10000));

	    }
    }

	void pop(Encoder &encoderData)
	{

		std::unique_lock<std::mutex> lock{ encoderQueueMutex };

		std::cout << "Debug@@@@@@@@@@@@@@@ quesesize: " << encoderQueue.size() << std::endl;

        if (encoderQueue.empty()){
			fetchVar.notify_one();
			popVar.wait_for(lock, std::chrono::milliseconds(1000000000000));
		}


		encoderData = encoderQueue.front();
        encoderQueue.pop_front();
		std::cout << "Debug@@@@@@@@@@@@@@@ PopValue: x:" << encoderData.x << "y: " << encoderData.y << "theta: " <<  encoderData.theta << std::endl;

		fetchVar.notify_one();
	}

    /**
	void pop(Frame &curFrame)
	{
        if (encoderQueue.empty()){
			fetchVar.notify_one();
		}

		std::unique_lock<std::mutex> lock{ encoderQueueMutex };

		Encoder encoderData = encoderQueue.front();

        curFrame.encoderX = encoderData.x;
        curFrame.encoderY = encoderData.y;
        curFrame.encoderTheta = encoderData.theta;

        encoderQueue.pop_front();

		fetchVar.notify_one();
	}
    */

	bool isEmpty()
	{
		std::unique_lock<std::mutex> lock{ encoderQueueMutex };
		return encoderQueue.empty();
	}


private:

	std::thread daemon;
    FILE* encoderReader;

    int counter;

    int queSize;
	std::mutex encoderQueueMutex;
    std::deque<Encoder> encoderQueue;

	std::condition_variable fetchVar;
	std::condition_variable popVar;

};
