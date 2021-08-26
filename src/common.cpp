/* 
Created by li on 24/8/2021.
Modified by Gu on 24/8/2021
 */

#include "common.h"

uint8_t Camera::cam_count_ = 0;

/* 
根据path，获取视频流中每一帧的时间戳，保存在动态数组std::vector<XSTime>里面。
 */
bool Camera::GetTimestampFromSei(std::string path,std::vector<XSTime> TV)
{   
    int time_index = 0;
       
	AVFormatContext *ifmt_ctx = NULL;
	AVPacket pkt;
	const char *in_filename;
	int ret, i;
	int videoindex = -1;
	int frame_index = 0;
	int64_t start_time = 0;

	in_filename = path.c_str();

	avformat_network_init(); //初始化网络库

    //avformat_open_input(&ifmt_ctx, in_filename, 0,0)；用来打开文件，解封文件头，若成功，返回0。
	if ((ret = avformat_open_input(&ifmt_ctx, in_filename, 0, 0)) < 0) {
		printf("Could not open input file.");
		goto end;
	}

    //avformat_find_stream_info(ifmt_ctx, 0)：用来获取音视频流的信息，若成功，返回0。
	if ((ret = avformat_find_stream_info(ifmt_ctx, 0)) < 0) {
		printf("Failed to retrieve input stream information");
		goto end;
	}

    //作用：寻找视频流。
    for(i=0;i<ifmt_ctx->nb_streams;i++)
    {
        if(ifmt_ctx->streams[i]->codecpar->codec_type==AVMEDIA_TYPE_VIDEO)  //如果是视频类型
        {
            videoindex = i;
            break;
        }
    }

	av_dump_format(ifmt_ctx, 0, in_filename, 0); //打印流信息


	start_time = av_gettime();

	while (1) {
		AVStream *in_stream;

		ret = av_read_frame(ifmt_ctx, &pkt);
		if (ret < 0){
			printf("read finished!\n");
			break;
		}

		if (pkt.stream_index == videoindex){
			frame_index++;

            //判断是否是SEI
            if ((pkt.data[4] & 0x1F) == 6)
            {
                int k=6;
                while(pkt.data[k-1]!=0x54 || pkt.data[k]!=0x4D)
                {
                    k++;
                }
                k++;
                XSTime xs_sei_time; 
                uint8_t datalenth = pkt.data[k];

                uint8_t xs_year1 = pkt.data[k+1];
                uint8_t xs_year2 = pkt.data[k+2];
                uint8_t xs_mon = pkt.data[k+3];

                xs_sei_time.day_ = pkt.data[k+4];
                xs_sei_time.hour_ = pkt.data[k+5];
                xs_sei_time.min_ = pkt.data[k+6];
                xs_sei_time.sec_ = pkt.data[k+7];

                uint8_t xs_msec1 = pkt.data[k+8];
                uint8_t xs_msec2 = pkt.data[k+9];

                memcpy(&xs_sei_time.msec_, pkt.data+k+8, 2);

                time_index++;
                TV.push_back(xs_sei_time);
            }

		}

	}
	 
end:
	avformat_close_input(&ifmt_ctx);
	 
	if (ret < 0 && ret != AVERROR_EOF) {
		printf("Error occurred.\n");
		return false;
	}

	return true;
}