#include "esp32s2_socket.h"
#include "esp32s2_buzzer.h"
#include "esp32s2_wifi.h"
#include "esp32s2_buff_operations.h"
#include "esp32s2_24lc512.h"
#include "esp32s2_gpio.h"
#include "esp32s2_main.h"
#include "esp32s2_task.h"
#include "string.h"
#include "esp32s2_fatal_err.h"

#define TAG "sockets"
#define TCPServerIP "192.168.1.12"
#define TCPClientIP "192.168.1.12"
#define PORT 5000
#define MAX_COUNT_TO_WAIT 110
#define USE_KEEP_ALIVE 1
#define MAX_SOCKET_FAIL_CNT 3

#define HACK_FOR_WINDOWS_TOOL 1
#define MAX_ALLOWED_OPEN_SOCKETS 5

uint16_t client_socket_id[MAX_ALLOWED_OPEN_SOCKETS];

extern uint8_t access_point;

/* Server details */
uint8_t https_password[SERV_IP_LEN];
uint8_t ap_ssid_buff[AP_SSID_LEN];
uint8_t ap_psw_buff[AP_PSW_LEN];
uint16_t server_port_address;

uint8_t config_calib_auth = 0;
uint32_t config_calib_ses_id;
uint32_t config_calib_ses_res;
uint8_t client_con_id, config_calib_mode;

uint8_t socket_client_status = 0;

uint8_t recv_buf[RECV_BUFF_SIZE], recv_buf_head = 0, recv_buf_tail = 0;
uint8_t signalled_to_stop_server = 0;
uint8_t ready_to_be_exited = 0;
uint8_t tcp_server_is_alive = 0;

uint8_t stream_rx_buff[UART_STREAM_RX_BUFF_SIZE];
uint8_t stream_tx_buff[UART_STREAM_TX_BUFF_SIZE];
uint8_t uart_cmd, uart_arg1;
uint16_t total_req_len;

int sock;
static int client_socket = 0;//client socket
static int master_socket = 0; 
int r;

extern uint8_t exit_server_task;

extern TaskHandle_t serverTask_handle;
extern TaskHandle_t clientTask_handle;


uint32_t turt_fm_hw_ver = ((HW_VERSION << 8) | (FW_VERSION));

/*
 * @Info:   Clear the receive buffer and close the socket;
 * @Param:  None
 * @Return: None
 */
void close_socket(void) {
    if(tcp_server_is_alive) {
        ESP_LOGI(TAG,"Closing the socket");
        uint16_t max_wait_cnt = 0;
        signalled_to_stop_server = 1;
        vTaskDelay(1000/portTICK_PERIOD_MS);
        bzero(recv_buf, sizeof(recv_buf));
        while(!ready_to_be_exited) { // Wait till gets signalled from tcp server
            vTaskDelay(100/portTICK_PERIOD_MS);
            if(max_wait_cnt++ >= MAX_COUNT_TO_WAIT) {
                break;
            }
        }
        shutdown(master_socket, 0);
        close(master_socket);
        master_socket = -1;
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

void deleteConfigCalibTask(void) {
    close_socket();
    signalled_to_stop_server = 0;
    ready_to_be_exited = 0;
    vTaskDelay(5);
    ESP_LOGI(TAG,"Stopping DHCP server");
    stop_dhcp_server();
    vTaskDelay(5);
    ESP_LOGI(TAG,"Stopping WiFi AP");
    esp_wifi_stop();
    vTaskDelay(5);
    extern uint8_t stopBatteryOverVoltageProtectionOnCalibration;
    stopBatteryOverVoltageProtectionOnCalibration = 0;
    if (config_calib_mode == CONFIG_CALIB_WIFI_SERVER) {
        if (serverTask_handle != NULL) {
            vTaskDelete(serverTask_handle);
            serverTask_handle = NULL;
            tcp_server_is_alive = 0;
            ESP_LOGI(TAG, "...tcp_server task deleted...");
        }
    } else {
        if (clientTask_handle != NULL) {
            vTaskDelete(clientTask_handle);
            clientTask_handle = NULL;
            ESP_LOGI(TAG, "...tcp_client task deleted...");
        }
    }
}

/*
 * @Info:  TCP client task; Socket allocation, binding, data receive;
 * @Param:  void task handler
 * @Return: None
 */
void tcp_client(void *pvParam) {
    config_calib_mode = CONFIG_CALIB_WIFI_CLIENT;
    ESP_LOGI(TAG, "tcp_client task started");
    struct sockaddr_in tcpServerAddr;
    tcpServerAddr.sin_addr.s_addr = inet_addr(TCPServerIP);
    tcpServerAddr.sin_family = AF_INET;
    tcpServerAddr.sin_port = htons(PORT);
    int r;
    sock = socket(AF_INET, SOCK_STREAM, 0);
    while (1) {
        if (xSemaphoreTake(connectionSemaphore, 10000 / portTICK_RATE_MS)) {
            while (1) {
                switch (socket_client_status) {
                    case idle:
                        sock = socket(AF_INET, SOCK_STREAM, 0);
                        if (sock < 0) {
                            ESP_LOGE(TAG, "... Failed to allocate socket...");
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                        } else {
                            ESP_LOGE(TAG, "... Allocated socket...");
                            socket_client_status = allocated_socket;
                        }
                        break;
                    case allocated_socket:
                        if (connect(sock, (struct sockaddr *) &tcpServerAddr, sizeof(tcpServerAddr)) != 0) {
                            socket_client_status = idle;
                            ESP_LOGE(TAG, "... socket connect failed errno=%d ...", errno);
                            close(sock);
                            vTaskDelay(5000 / portTICK_PERIOD_MS);
                        } else {
                            ESP_LOGI(TAG, "Connected to server");

                            for (int i = 0; i < 30; i++) {
                                stream_tx_buff[i] = i;
                            }
                            int err = send(sock, stream_tx_buff, 30, 0);
                            if (err < 0) {
                                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                            }
                            socket_client_status = connected_socket;
                            blink(led_magenta_color, FULL_ON);
                        }
                        break;
                    case connected_socket:
                        bzero(recv_buf, sizeof(recv_buf));
                        r = read(sock, recv_buf, sizeof(recv_buf) - 1);
                        if (r < 0) {
                            ESP_LOGI(TAG, "Socket closed");
                            socket_client_status = allocated_socket;
                        } else {
                            ESP_LOGI(TAG, "Checking data");
                            if (read_calib_config_req(&uart_cmd, stream_tx_buff, &total_req_len)) {
                                socket_client_status = data_received;
                                ESP_LOGI(TAG, "Data received");
                                extern uint8_t configuration_timeout_cnt;
                                configuration_timeout_cnt = 0;
                            }
                        }
                        break;
                    case data_received:
                        process_labview_cmd();
                        socket_client_status = connected_socket;
                        break;
                }
            }
        } else {
            extern uint8_t server_connect_retry_cnt;
            extern uint8_t server_connect_retry;
            if (server_connect_retry_cnt > 15) {
                server_connect_retry_cnt = 0;
                server_connect_retry += 1;
                esp_wifi_stop();
                ESP_LOGI(TAG, "Retrying connecting to E24by7");
                play_tone1();
                connectSTA("E24by7", "Energy24by7");
            }
            if (server_connect_retry >= 3) {
                server_connect_retry = 0;
                server_connect_retry_cnt = 0;
                set_algo_status(0);
                extern uint16_t watchdog_timer_stop;
                watchdog_timer_stop = 0;
                ESP_LOGI(TAG, "...tcp_client task deleted...");
                esp_wifi_stop();
                vTaskDelete(NULL);
            }

        }
    }
}

/*
 * @Info:  TCP server task; Socket allocation, binding, data receive;
 * @Param:  void task handler
 * @Return: None
 */
void tcp_server(void *pvParam) {
    extern uint8_t configuration_timeout_cnt;
    configuration_timeout_cnt = 0;
    config_calib_mode = CONFIG_CALIB_WIFI_SERVER;
    ESP_LOGI(TAG, "tcp_server task started \n");
    struct sockaddr_in tcpServerAddr;
    memset(&tcpServerAddr,0,sizeof(tcpServerAddr));
    tcpServerAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    tcpServerAddr.sin_family = AF_INET;
    tcpServerAddr.sin_port = htons(5000);
    struct sockaddr_in remote_addr;
    memset(&remote_addr,0,sizeof(remote_addr));
    unsigned int socklen;
    socklen = sizeof(remote_addr);
    socket_client_status = idle;

    //set of socket descriptors
    fd_set readfds;
    int max_sd,sd,valread = 0;
    int activity = 0;
    BaseType_t xTrueValue = pdTRUE;
    uint8_t bind_fail_cnt = 0;
    uint8_t listen_fail_cnt = 0;
    uint8_t select_err_cnt = 0;
    static uint8_t open_socket_cnt = 0;
    uint8_t i = 0;


#if USE_KEEP_ALIVE
    int keepAlive = 1;
    int keepIdle = 5;
    int keepInterval = 5;
    int keepCount = 3;
#endif
    
    master_socket = 0;
    client_socket = 0;
    ESP_LOGI(TAG, "TCP Server starting");

    master_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (master_socket < 0) {
        ESP_LOGE(TAG, "Failed to allocate socket: %d Error: %s", master_socket, strerror(errno));
    } else {
        socket_client_status = allocated_socket;
    }

    blink(led_magenta_color, FULL_ON);
    while (1) {
        if (xSemaphoreTake(serverSemaphore, 10000 / portTICK_RATE_MS)) {
            ESP_LOGI(TAG, "Entering TCP server function");
            while (1) {
                tcp_server_is_alive = 1;
                if(!signalled_to_stop_server) {
                    switch (socket_client_status) {
                        case idle:
                            master_socket = socket(AF_INET, SOCK_STREAM, 0);
                            if (master_socket < 0) {
                                perror("socket");
                                ESP_LOGE(TAG, "... Failed to allocate socket errno=%d", errno);
                                vTaskDelay(1000 / portTICK_PERIOD_MS);
                            } else {
                                ESP_LOGE(TAG, "... Allocated socket...");
                                socket_client_status = allocated_socket;
                            }
                            break;
                        case allocated_socket:
                            setsockopt(master_socket, SOL_SOCKET, SO_REUSEADDR, ( void * ) &xTrueValue, sizeof( xTrueValue ));
                            if (bind(master_socket, (struct sockaddr *) &tcpServerAddr, sizeof(tcpServerAddr)) != 0) {
                                socket_client_status = idle;
                                perror("bind");
                                ESP_LOGE(TAG, "... socket bind failed errno=%d ...", errno);
                                shutdown(master_socket,0);
                                close(master_socket);
                                vTaskDelay(1000 / portTICK_PERIOD_MS);
                                if( bind_fail_cnt++ >= MAX_SOCKET_FAIL_CNT) {
                                    ESP_LOGI("TCP_SERVER","bind fails .. Restarting the system");
                                    fatal_err_handler(TCP_SERVER,BIND_FAILS);
                                }
                            } else {
                                ESP_LOGI(TAG, "Socket bind successfull...");
                                socket_client_status = bind_socket;
                            }
                            break;
                        case bind_socket:
                            if (listen(master_socket, 3) != 0) {
                                perror("listen");
                                ESP_LOGE(TAG, "... socket listen failed errno=%d ", errno);
                                shutdown(master_socket,0);
                                close(master_socket);
                                master_socket = 0;
                                vTaskDelay(1000 / portTICK_PERIOD_MS);
                                socket_client_status = idle;
                                if( listen_fail_cnt++ >= MAX_SOCKET_FAIL_CNT) {
                                    ESP_LOGI("TCP_SERVER","listen fails .. Restarting the system");
                                    fatal_err_handler(TCP_SERVER,LISTEN_FAILS);
                                }
                            } else {
                                ESP_LOGI(TAG, "Socket listening with backlog 3");
                                socket_client_status = listen_socket;
                            }
                            break;
                        case listen_socket:
#ifndef HACK_FOR_WINDOWS_TOOL
                            client_socket = 0;
#endif
                            ESP_LOGI(TAG, "Waiting for new socket connection ...");
                            //clear the socket set
                            FD_ZERO(&readfds);

                            //add master socket to set
                            struct timeval tv;

                            /* Wait up to five seconds. */
                            tv.tv_sec = 5;
                            tv.tv_usec = 0;
                            
                            FD_CLR(master_socket, &readfds);
                            FD_SET(master_socket, &readfds);
                            max_sd = master_socket;
                            sd = client_socket;
                            //if valid socket descriptor then add to read list
                            if(sd > 0) {
                                FD_CLR( sd, &readfds);
                                FD_SET( sd , &readfds);
                            }

                            //highest file descriptor number, need it for the select function
                            if(sd > max_sd) {
                                max_sd = sd;
                            }

                            //wait for an activity on one of the sockets , timeout is 5 sec
                            activity = select( max_sd + 1 , &readfds , NULL , NULL , &tv);

                            if ((activity < 0) && (errno!=EINTR))
                            {
                                ESP_LOGI(TAG, "select error");
                                socket_client_status = bind_socket;
                                if(select_err_cnt++ >= MAX_SOCKET_FAIL_CNT) {
                                    ESP_LOGI("TCP_SERVER","select fails .. Restarting the system");
                                    fatal_err_handler(TCP_SERVER,SELECT_FAILS);
                                }
                            }
                            //If something happened on the master socket ,
                            //then its an incoming connection
                            if (FD_ISSET(master_socket, &readfds))
                            {
                                client_socket = accept(master_socket, (struct sockaddr *) &remote_addr, &socklen);
                                if(client_socket < 0) {
                                    perror("accept");
                                    shutdown(client_socket,0);
                                    close(client_socket);
                                    client_socket = 0;
                                    ESP_LOGI("TCP_SERVER","accept fails .. Restarting the system");
                                    fatal_err_handler(TCP_SERVER,ACCEPT_FAILS);
                                } else {
#if USE_KEEP_ALIVE
                                    // Set tcp keepalive option
                                    setsockopt(client_socket, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
                                    setsockopt(client_socket, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
                                    setsockopt(client_socket, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
                                    setsockopt(client_socket, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
#endif

                                    ESP_LOGI(TAG, "New connection request from : %s:%d",
                                            inet_ntoa(remote_addr.sin_addr.s_addr),
                                            ntohs(remote_addr.sin_port));

                                    //fcntl(client_socket, F_SETFL, O_NONBLOCK);
                                   socket_client_status = connected_socket;
                                }
                            } else { //else its some IO operation on some other socket
                                sd = client_socket;
                                if (FD_ISSET( sd , &readfds))
                                {
                                    //Check if it was for closing , and also read the
                                    //incoming message
                                    memset(recv_buf,0, sizeof(recv_buf));
                                    //valread = read( sd , recv_buf, sizeof(recv_buf) - 1);
                                    valread = recv( client_socket , recv_buf, sizeof(recv_buf) - 1,0);
                                    if (valread  == 0)
                                    {
                                        //Somebody disconnected , get his details and print
                                        getpeername(client_socket , (struct sockaddr*)&remote_addr , \
                                                (socklen_t*)&socklen);
                                        ESP_LOGI(TAG, "Client disconnected , ip %s , port %d \n" ,
                                                inet_ntoa(remote_addr.sin_addr) , ntohs(remote_addr.sin_port));
                                        //Close the socket and mark as 0 in list for reuse
                                        shutdown(client_socket,0);
                                        close( client_socket );
                                        sd = 0;
                                        client_socket = 0;
#if 0
                                        //Close the socket and mark as 0 in list for reuse
                                        shutdown(client_socket,0);
                                        close( client_socket );
                                        client_socket = 0;
                                        socket_client_status = bind_socket;
#endif
                                    } else if (valread < 0) {
                                        perror("recv");
                                        sd = 0;
                                        shutdown(client_socket,0);
                                        close(client_socket);
                                        client_socket = 0;
#if 0
                                        shutdown(client_socket,0);
                                        close(client_socket);
                                        client_socket = 0;
                                        socket_client_status = bind_socket;
#endif
                                    } else {
                                        ESP_LOGI(TAG, "Checking data :: recv_buf[0]: %02X", recv_buf[0]);
                                        if (read_calib_config_req(&uart_cmd, stream_tx_buff, &total_req_len)) {
                                            socket_client_status = data_received;
                                            ESP_LOGI(TAG, "Data received.");
                                            extern uint8_t configuration_timeout_cnt;
                                            configuration_timeout_cnt = 0;
                                        }
                                    }

                                }
#ifndef HACK_FOR_WINDOWS_TOOL
                                shutdown(client_socket,2);
                                close(client_socket);
                                client_socket = 0;
#endif
                            }
                            break;
                        case connected_socket:
                            memset(recv_buf,0, sizeof(recv_buf));
                            r = recv(client_socket, recv_buf, sizeof(recv_buf) - 1, 0);

                            if (r == 0) {
                                ESP_LOGI(TAG, "Client disconnected");
                                shutdown(client_socket,0);
                                close(client_socket);
                                client_socket = 0;
                                //socket_client_status = bind_socket;
                            } else if (r < 0) {
                                perror("recv");
                                ESP_LOGI(TAG, "Read error: (%d)" , r);
                                shutdown(client_socket,0);
                                close(client_socket);
                                client_socket = 0;
                                //socket_client_status = bind_socket;
                            } else {
                                ESP_LOGI(TAG, "Checking data : recv_buf[0]: %02X", recv_buf[0]);
                                if (read_calib_config_req(&uart_cmd, stream_tx_buff, &total_req_len)) {
                                    socket_client_status = data_received;
                                    ESP_LOGI(TAG, "Data received");
                                    extern uint8_t configuration_timeout_cnt;
                                    configuration_timeout_cnt = 0;
                                }
                            }
                            break;
                        case data_received:
                            process_labview_cmd();
                            /* Close the client socket connection */
#ifdef HACK_FOR_WINDOWS_TOOL
                            /* 
                             *To support both windows and mobile app can't close every time
                             * Hence max open socket count reaches a max count closing the sockets
                             * Improper way, but no other choice. God bless
                             */
                            client_socket_id[open_socket_cnt] = client_socket;
                            open_socket_cnt++;
                            if(open_socket_cnt > MAX_ALLOWED_OPEN_SOCKETS) {
                                for(i=0; i < MAX_ALLOWED_OPEN_SOCKETS; i++) {
                                    shutdown(client_socket_id[i],0);
                                    close(client_socket_id[i]);
                                    client_socket_id[i] = 0;
                                    vTaskDelay(100/portTICK_PERIOD_MS);
                                }
                                client_socket = 0;
                                open_socket_cnt = 0;
                            }
#else 
                                shutdown(client_socket,0);
                                close(client_socket);
                                client_socket = 0;
#endif
                            socket_client_status = listen_socket;
                            break;
                        case do_nothing:
                            break;
                    }
                } else {
                    ready_to_be_exited = 1;
                    while(1) {
                        vTaskDelay(10/portTICK_PERIOD_MS);
                    }
                }
            }
        }
    }
}

/*
 * @Info:   General CRC16 calculation
 * @Param:  Data buffer and length of the data buffer
 * @Return: crc16
 */
uint16_t crc_16(uint8_t *data_p, uint16_t length) {
    uint8_t i;
    uint16_t data;
    uint16_t crc = 0xffff;
    if (length == 0) {
        return (~crc);
    }
    do {
        for (i = 0, data = ((uint16_t) 0xff & *data_p++);
             i < 8;
             i++, data >>= 1) {
            if ((crc & 0x0001) ^ (data & 0x0001))
                crc = (crc >> 1) ^ POLY;
            else
                crc >>= 1;
        }
    } while (--length);
    crc = ~crc;
    data = crc;
    crc = ((crc << 8) | ((data >> 8) & 0xff));
    return (crc);
}

/*
 * @Info:   Parse incoming data from the tool
 * @Param:  None
 * @Return: None
 */

uint8_t read_calib_config_req(uint8_t *req_cmd, uint8_t *req_payload, uint16_t *req_pkt_len) {
    uint8_t _sync_word, buff_idx;
    uint16_t _full_pkt_len, _check_sum, _calc_check_sum;
    /* Extract the sync word from the received buffer */
    _sync_word = recv_buf[0];
    if (_sync_word == CONFIG_CALIB_SYNC) {
        for (buff_idx = 0; buff_idx < REQ_HEADER_LEN; buff_idx++) {
            /* Since sync word is already extracted copy the bytes starting from command into the rx_buffer */
            stream_rx_buff[buff_idx] = recv_buf[buff_idx + 1];
        }
        *req_cmd = stream_rx_buff[REQ_ID_IDX];
        *req_pkt_len = ((stream_rx_buff[REQ_DAT_LEN_IDX + 1] << 8) | (stream_rx_buff[REQ_DAT_LEN_IDX]));
        memcpy((uint8_t * ) & config_calib_ses_res, &stream_rx_buff[REQ_SES_ID_IDX], SES_ID_LEN);
        if (*req_pkt_len > MAX_PAYLOAD_LEN) {
            ESP_LOGI(TAG, "Payload greater than maximum length");
            return 0;
        }
        _full_pkt_len = *req_pkt_len + REQ_HEADER_LEN + REQ_CRC_LEN;

        for (buff_idx = REQ_PAYLOAD_IDX; buff_idx < _full_pkt_len; buff_idx++) {
            stream_rx_buff[buff_idx] = recv_buf[buff_idx + 1];
        }
        _full_pkt_len -= REQ_CRC_LEN;
        _calc_check_sum = crc_16(stream_rx_buff, (_full_pkt_len));
        _check_sum = ((stream_rx_buff[_full_pkt_len + 1] << 8) | stream_rx_buff[_full_pkt_len]);

        if (*req_pkt_len > 0) {
            memcpy(req_payload, &stream_rx_buff[REQ_PAYLOAD_IDX], *req_pkt_len);
        }
        ESP_LOGI(TAG, "Read calib/config returned success");
        return 1;
    }
    ESP_LOGI(TAG, "Sync word didn't match");
    return 0;

}

/*
 * @Info:   Send response to utility
 * @Param:  None
 * @Return: None
 */

void res_to_utility(uint8_t _res_type, uint8_t *_res_buff, uint16_t _res_len) {
    uint16_t _full_pkt_len, check_sum;
    uint8_t i = 0;
    if (config_calib_mode == CONFIG_CALIB_WIFI_CLIENT) {
        ESP_LOGI(TAG, "Response to utility as a client");
        stream_tx_buff[RES_SYNC_IDX] = CONFIG_CALIB_SYNC;
        stream_tx_buff[REQ_SES_ID_IDX] = _res_type;
        stream_tx_buff[RES_DAT_LEN_IDX] = (_res_len & 0xFF);
        stream_tx_buff[RES_DAT_LEN_IDX + 1] = ((_res_len >> 8) & 0xFF);
        memcpy((uint8_t * ) & stream_tx_buff[RES_SSID_IDX], &config_calib_ses_res, SES_ID_LEN);
        if (_res_len > 0) {
            memcpy(&stream_tx_buff[RES_PAYLOAD_IDX_CLIENT], _res_buff, _res_len);
        }
        _full_pkt_len = RES_HEADER_LEN_CLIENT + _res_len;
        check_sum = crc_16(stream_tx_buff, _full_pkt_len);
        stream_tx_buff[_full_pkt_len + 1] = (check_sum >> 8);
        stream_tx_buff[_full_pkt_len] = (check_sum & 0xFF);
        _full_pkt_len += RES_CRC_LEN;
        int err = send(sock, stream_tx_buff, _full_pkt_len, 0);
        if (err < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        }
    } else {
        ESP_LOGI(TAG, "Response to utility as a server");
        stream_tx_buff[RES_SYNC_IDX] = CONFIG_CALIB_SYNC;
        stream_tx_buff[REQ_SES_ID_IDX] = _res_type;
        stream_tx_buff[RES_DAT_LEN_IDX] = (_res_len & 0xFF);
        stream_tx_buff[RES_DAT_LEN_IDX + 1] = ((_res_len >> 8) & 0xFF);
        if (_res_len > 0) {
            memcpy(&stream_tx_buff[RES_PAYLOAD_IDX], _res_buff, _res_len);
        }
        _full_pkt_len = RES_HEADER_LEN + _res_len;
        check_sum = crc_16(stream_tx_buff, _full_pkt_len);
        stream_tx_buff[_full_pkt_len + 1] = (check_sum >> 8);
        stream_tx_buff[_full_pkt_len] = (check_sum & 0xFF);
        _full_pkt_len += RES_CRC_LEN;
        // Sometimes device not able to capture the packet.
        // Hence trying 5 retries
        //for(i = 0; i < 5; i++) { // WINDOWS HACK
            int err = write(client_socket, stream_tx_buff, _full_pkt_len);
            if (err < 0) {
                perror("write");
                ESP_LOGE(TAG, "Error occurred during sending %s: errno %d", __func__,errno);
            }
            vTaskDelay(10/portTICK_PERIOD_MS);
        //}
    }
    beep(1, SHORT_BEEP);
}

/*
 * @Info:   Send response to utility for authentication
 * @Param:  None
 * @Return: None
 */
void
send_con_res_config_calib() {
    extern uint8_t dev_type;
    if ((i2c_eeprom_read_int32(EE_CALIB_DONE_ADDR) == CALIB_CONFIG_DONE_VALUE)) {
        stream_rx_buff[CON_RES_FLAG_IDX] = 1;
    } else {
        stream_rx_buff[CON_RES_FLAG_IDX] = 0;
    }
    i2c_eeprom_read_buff(EE_DEV_SER_NO_ADDR, &stream_rx_buff[CON_RES_ID_IDX], (DEV_SER_NO_LEN + DEVICE_TYPE_LEN));
    memcpy(&stream_rx_buff[CON_RES_ID_IDX + DEV_SER_NO_LEN], &dev_type, DEVICE_TYPE_LEN);
    memcpy(&stream_rx_buff[CON_RES_HW_VER_IDX], (uint8_t * ) & turt_fm_hw_ver, HW_FM_VER_LEN);
    stream_rx_buff[CON_RES_PROT_VER_IDX] = CALIB_CONFIG_MAJ_VER;
    stream_rx_buff[CON_RES_PROT_VER_IDX + 1] = CALIB_CONFIG_MIN_VER;
    memcpy(&stream_rx_buff[CON_RES_SID_IDX], (uint8_t * ) & config_calib_ses_id, SES_ID_LEN);
    res_to_utility(REQ_CON, stream_rx_buff, CON_RES_PKT_LEN);
}

/*
 * @info:   Read digital value from the device
 * @Param:  None
 * @Return: None
 */
void read_send_adc_value() {
    read_adc_all(stream_rx_buff);
    res_to_utility(CMD_GET_ADC, stream_rx_buff, 20 + 8);
}

/*
 * @Info:   Read values from EEPROM
 * @Param:  None
 * @Return: None
 */
void read_send_eeprom_val() {
    uint16_t _ee_start_address;
    uint8_t _ee_dat_len;
    memcpy((uint8_t * ) & _ee_start_address, &stream_tx_buff[0], 2);
    _ee_dat_len = stream_tx_buff[2];
    if ((_ee_dat_len > 0) && (_ee_dat_len <= MAX_PAYLOAD_LEN)) {
        memcpy(&stream_rx_buff[0], (uint8_t * ) & _ee_start_address, 2);
        stream_rx_buff[2] = _ee_dat_len;
        i2c_eeprom_read_buff(_ee_start_address, &stream_rx_buff[3], _ee_dat_len);
        res_to_utility(CMD_READ_EEPROM, stream_rx_buff, (_ee_dat_len + 3));
    }
    if (_ee_start_address >= 0xFF21) {
        clear_system_flag(SF_USEABLE_SOC_RC);
    }
}

/*
 * @Info:   Load WiFi parameters
 * @Param:  SSID and PSW
 * @Return: None
 */
void load_wifi_param() {
    i2c_eeprom_read_buff(64672, https_password, SERV_IP_LEN);
    // server_port_address = i2c_eeprom_read_int16(EE_SERV_PORT_ADDR);
    i2c_eeprom_read_buff(EE_AP_SSID_ADDR, ap_ssid_buff, AP_SSID_LEN);
    i2c_eeprom_read_buff(EE_AP_PSW_ADDR, ap_psw_buff, AP_PSW_LEN);
    https_password[SERV_IP_LEN - 1] = '\0';
    ap_ssid_buff[AP_SSID_LEN - 1] = '\0';
    ap_psw_buff[AP_PSW_LEN - 1] = '\0';
    access_point = primary;
    ESP_LOGI(TAG, "HTTP password at 0xFCA0- %s\n", (char *) https_password);

    uint8_t https_FC_password[SERV_IP_LEN];
    i2c_eeprom_read_buff(0xFC00, https_FC_password, SERV_IP_LEN);
    ESP_LOGI(TAG, "HTTP password at 0xFC00 - %s\n", (char *) https_FC_password);
}

/*
 * @Info:   Calcualte CRC for data to be stored in EEPROM
 * @Param:  Memory starting address, Total length, memory location to store CRC
 * @Return: CRC16
 */
uint16_t calc_update_eeprom_crc(uint16_t _ee_start_address, uint8_t _ee_len, uint16_t _crc_addr, uint8_t is_update) {
    uint16_t _ee_crc = 0;
    vTaskDelay(100 / portTICK_RATE_MS);
    i2c_eeprom_read_buff(_ee_start_address, stream_rx_buff, _ee_len);
    _ee_crc = crc_16(stream_rx_buff, _ee_len);
    if (is_update) {
        vTaskDelay(100 / portTICK_RATE_MS);
        ESP_LOGI("EEPROM_WRITE", "WRITING INTO EE_CONFIG/CALIB_CRC_ADDR WITH CRC : %04X",_ee_crc);
        i2c_eeprom_write_int16(_crc_addr, _ee_crc);
    }
    return (_ee_crc);
}

/*
 * @Info:   Write data into EEPROM
 * @param:  None
 * @Return: None
 */
void write_const_to_eeprom() {
    uint16_t _ee_start_address, _ee_end_address;
    uint8_t _ee_dat_len;
    memcpy((uint8_t * ) & _ee_start_address, &stream_tx_buff[0], 2);
    _ee_dat_len = stream_tx_buff[2];
    _ee_end_address = (_ee_start_address + _ee_dat_len) - 1;

    if ((_ee_dat_len > 0) && (_ee_dat_len <= MAX_PAYLOAD_LEN) &&
        (_ee_start_address >= EE_CONFIG_DAT_START_ADDR && _ee_end_address <= EE_SYS_THR_END_ADDR)) {
        i2c_eeprom_write_buff(_ee_start_address, &stream_tx_buff[3], _ee_dat_len);
        ESP_LOGI(TAG, "%d written into EEPROM", _ee_dat_len);
        vTaskDelay(20 / portTICK_RATE_MS);
        memcpy(&stream_rx_buff[0], (uint8_t * ) & _ee_start_address, 2);
        stream_rx_buff[2] = _ee_dat_len;
        i2c_eeprom_read_buff(_ee_start_address, &stream_rx_buff[3], _ee_dat_len);
#if 0
        for (int i = 0; i < _ee_dat_len; i++) {
            ESP_LOGI(TAG, "Element %d in data is %d", i, stream_rx_buff[i]);
        }
#endif
        ESP_LOGI(TAG, "Preparing response to utility");
        res_to_utility(CMD_WRITE_EEPROM, stream_rx_buff, (_ee_dat_len + 3));
        beep(1, SHORT_BEEP);
    }

    vTaskDelay(100 / portTICK_RATE_MS);
    if (_ee_start_address >= EE_CONFIG_DAT_START_ADDR && _ee_end_address < EE_DC_OVER_I_THR_ADDR) {
        ESP_LOGI("EEPROM_WRITE", "WRITING INTO EE_CONFIG_DONE_ADDR");
        i2c_eeprom_write_int32(EE_CONFIG_DONE_ADDR, CALIB_CONFIG_DONE_VALUE);
        ESP_LOGI(TAG, "Configuration done address");
        vTaskDelay(100 / portTICK_RATE_MS);
    }

    if (_ee_start_address >= EE_DC_OVER_I_THR_ADDR && _ee_end_address < EE_CONFIG_DONE_ADDR) {
        ESP_LOGI("EEPROM_WRITE", "WRITING INTO EE_CONFIG_DONE_ADDR - DEBUG 2");
        i2c_eeprom_write_int32(EE_CONFIG_DONE_ADDR, CALIB_CONFIG_DONE_VALUE);
        vTaskDelay(100 / portTICK_RATE_MS);
        clear_error(ERR_NOT_CONFIG);
        ESP_LOGI(TAG, "Clear configuration error");
    }

    if (_ee_start_address >= EE_CONFIG_DAT_START_ADDR && _ee_end_address < EE_CONFIG_DONE_ADDR) {
        ESP_LOGI("EEPROM_WRITE", "WRITING INTO EE_CONFIG_CRC_ADDR");
        calc_update_eeprom_crc(EE_CONFIG_DAT_START_ADDR, CONFIG_PARAM_LEN, EE_CONFIG_CRC_ADDR, 1);
        ESP_LOGI(TAG, "Calculating CRC");
    }

    if (_ee_start_address >= EE_CALIB_DAT_START_ADDR && _ee_end_address < EE_CALIB_DONE_ADDR) {
        ESP_LOGI("EEPROM_WRITE", "WRITING INTO EE_CALIB_DONE_ADDR");
        i2c_eeprom_write_int32(EE_CALIB_DONE_ADDR, CALIB_CONFIG_DONE_VALUE);
        ESP_LOGI("EEPROM_WRITE", "WRITING INTO EE_CALIB_CRC_ADDR");
        calc_update_eeprom_crc(EE_CALIB_DAT_START_ADDR, CALIB_CONST_LEN, EE_CALIB_CRC_ADDR, 1);
        clear_error(ERR_NOT_CALIB);
        ESP_LOGI(TAG, "Calibration done address");
    }

    vTaskDelay(100 / portTICK_RATE_MS);
    load_sensor_constants();
    load_wifi_param();
}

/*
 * @Info:   Send analog values to utility
 * @Param:  None
 * @Return: None
 */
void send_real_values() {
    extern uint8_t mains;
    extern float inverter_vol, temperature1, temperature2, vol_ref;
    get_sensor_values(FLUSH_ADC_VAL, 0);
    memcpy(&stream_rx_buff[0], (uint8_t * ) & algo_param.cur_bat_v, 4);
    memcpy(&stream_rx_buff[4], (uint8_t * ) & algo_param.cur_chg_i, 4);
    memcpy(&stream_rx_buff[8], (uint8_t * ) & algo_param.cur_dis_i, 4);
    memcpy(&stream_rx_buff[12], (uint8_t * ) & algo_param.cur_sol_v, 4);
    memcpy(&stream_rx_buff[16], (uint8_t * ) & algo_param.cur_sol_i, 4);
    memcpy(&stream_rx_buff[20], (uint8_t * ) & algo_param.cur_ac_load_i, 4);
    memcpy(&stream_rx_buff[24], (uint8_t * ) & inverter_vol, 4);
    memcpy(&stream_rx_buff[28], (uint8_t * ) & temperature1, 4);
    memcpy(&stream_rx_buff[32], (uint8_t * ) & vol_ref, 4);
    memcpy(&stream_rx_buff[36], (uint8_t * ) & mains, 1);
    memcpy(&stream_rx_buff[37], (uint8_t * ) & temperature2, 4);
    res_to_utility(CMD_GET_REAL_VAL, stream_rx_buff, 41);
}

/*
 * @Info:   The code is used to perform the following
 *          1. Authentication
 *          2. Set time
 *          3. Get ADC value
 *          4. Get swtich status
 *          5. Set switch status
 *          6. Test connectivity with server
 *          7. Read  from EEPROM
 *          8. Write to   EEPROM 
 *          9. Get real values
 *          10. Buzzer sound
 *          11. Exit calibration/configuration mode
 * @Param:  None
 * @Return: None
 */
void process_labview_cmd() {
    switch (uart_cmd) {
        /* Authentication */
        case REQ_CON:
            ESP_LOGI("inside_process_labview_cmd", "Authentication request command");
            send_con_res_config_calib();
            break;
            /* Get ADC value */
        case CMD_GET_ADC:
            ESP_LOGI("inside_process_labview_cmd", "Get ADC command");
            read_send_adc_value();
            break;
            /* Get the status of device state */
        case CMD_GET_SW:
            ESP_LOGI("inside_process_labview_cmd", "Get switch commmand");
            stream_rx_buff[0] = switch_state;
            res_to_utility(CMD_GET_SW, stream_rx_buff, 1);
            break;
            /* Set the state of the device */
        case CMD_SET_SW:
            ESP_LOGI("inside_process_labview_cmd", "Set switch status command");
            turtle_set_sw(stream_tx_buff[0]);
            stream_rx_buff[0] = switch_state;
            res_to_utility(CMD_SET_SW, stream_rx_buff, 1);
            break;
            /* Test connectivity */
        case CMD_TEST_CONFIG:
            ESP_LOGI("inside_process_labview_cmd", "Test configuration command");
            res_to_utility(CMD_TEST_CONFIG, stream_rx_buff, 0);
            load_wifi_param();
            xSemaphoreGive(testConnectionSemaphore);
            break;
            /* Read EEPROM */
        case CMD_READ_EEPROM:
            ESP_LOGI("inside_process_labview_cmd", "EEPROM read command");
            read_send_eeprom_val();
            break;
            /* Write EEPROM */
        case CMD_WRITE_EEPROM:
            ESP_LOGI("inside_process_labview_cmd", " EEPROM write command");
            write_const_to_eeprom();
            break;
            /* Get sensor values from device */
        case CMD_GET_REAL_VAL:
            ESP_LOGI("inside_process_labview_cmd", "Sensor value command");
            send_real_values();
            break;
        case CMD_EXIT:
            /* Exit from task */
            ESP_LOGI("inside_process_labview_cmd", "Exit command received");
            res_to_utility(CMD_EXIT, stream_rx_buff, 0);
            break;
        default:
            ESP_LOGI("inside_process_labview_cmd", "Wrong command received");
            break;
    }
}
