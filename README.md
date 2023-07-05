note: 
1、ble  b server  发送“peidui”
2、本体是服务端，esp32是客户端；  esp32是服务端，app是客户端       谁有屏幕谁做客户端去连接
3、增加ota功能



/*************************************消息邮箱*************************************************************/
#if 0
    EventGroupHandle_t g_Event_Handle = NULL;
    QueueHandle_t Tcp_To_Sdio_Queue_Handle, Sdio_to_Tcp__Queue_Handle;
     //creat event group
    g_Event_Handle = xEventGroupCreate();    //时间组没有使用
    if(g_Event_Handle == NULL)
    {
        printf("g_Event_Handle error");
    }

    Tcp_To_Sdio_Queue_Handle = xQueueCreate(1,4096);     //升级的字节4096
    if(Tcp_To_Sdio_Queue_Handle == NULL)
    {
         printf("Tcp_To_Sdio_Queue_Handle error");
    }

    Sdio_to_Tcp__Queue_Handle = xQueueCreate(1,128);      //linux 板子回复消息字节128
    if(Sdio_to_Tcp__Queue_Handle == NULL)
    {
         printf("Sdio_to_Tcp__Queue_Handle error");
    }
#endif

/*************************************freertos 示例************************************************/
#if 0
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);/* code */
        #if 0
        EventBits_t r_event;                                    /* 定义一个事件接收变量 */ 
                r_event = xEventGroupWaitBits(g_Event_Handle,     /* 事件对象句柄 */ 
                              BIT0,                        /* 接收任务感兴趣的事件 */ 
                              pdTRUE,                           /* 退出时清除事件位 */ 
                              pdTRUE,                           /* 满足感兴趣的所有事件 */ 
                              portMAX_DELAY);                   /* 指定超时事件,一直等 */ 
  
        if (r_event == BIT0) 
        { 
            /* 如果接收完成并且正确 */ 
            printf ( "BIT0 BIT1 RECEIVE_CARLL\n"); 
        } 
        else 
        {
            printf ( "事件错误！\n"); 
        }
        #endif

        #if 0
        char rx_buffer[128];
        if( pdTRUE == xQueueReceive(Tcp_To_Sdio_Queue_Handle,rx_buffer,100) )
        {
            //printf("tcp send queue susessful");
            ESP_LOGI("carll", "接收到 queue susessful %c",rx_buffer[1]);
        }
        //ESP_LOGI("carll","接收到 queue fail\n");
        if(rx_buffer[1] == '2')
        {
            if( pdTRUE == xQueueSend(Sdio_to_Tcp__Queue_Handle,rx_buffer,0) )  //发送不会阻塞 carll
            {
                ESP_LOGI("carll","发送 queue sucessful\n");
                memset(rx_buffer,0,128);
            }   
            //ESP_LOGI("carll","发送 queue fail\n");
        }       
        #endif
    }
#endif