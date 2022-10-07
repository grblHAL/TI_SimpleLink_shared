//
// enet.c - lwIP/FreeRTOS TCP/IP stream implementation
//
// v1.4 / 2022-09-13 / Io Engineering / Terje
//

/*

Copyright (c) 2018-2022, Terje Io
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may
be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "shared/base/driver.h"

#if ETHERNET_ENABLE

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"

#include "utils/lwiplib.h"
//#include "utils/locator.h" // comment in to enable TIs locator service

#include "posix/sys/socket.h"

#include "grbl/report.h"
#include "grbl/nvs_buffer.h"

#include "networking/networking.h"

#define SYSTICK_INT_PRIORITY    0x80
#define ETHERNET_INT_PRIORITY   0xC0

// some defines for old lwIP stack

#ifndef IPADDR_STRLEN_MAX
#define IPADDR_STRLEN_MAX 16
#endif

#ifndef ip4addr_ntoa_r
#define ip4addr_ntoa_r(ipaddr, buf, buflen) ipaddr_ntoa_r(ipaddr, buf, buflen)
#endif

#ifndef ip4addr_aton
#define ip4addr_aton(cp, addr) ipaddr_aton(cp, addr)
#endif

//

static volatile bool linkUp = false;
static uint32_t IPAddress = 0;
static stream_type_t active_stream = StreamType_Null;
static network_settings_t network, ethernet;
static network_services_t services = {0}, allowed_services;
static uint32_t nvs_address;
static on_report_options_ptr on_report_options;
static on_stream_changed_ptr on_stream_changed;
static char netservices[NETWORK_SERVICES_LEN] = ""; // must be large enough to hold all service names

static char *enet_ip_address (void)
{
    static char ip[IPADDR_STRLEN_MAX];

    ip4addr_ntoa_r((const ip_addr_t *)&IPAddress, ip, IPADDR_STRLEN_MAX);

    return ip;
}

network_info_t *networking_get_info (void)
{
    static network_info_t info;

    memcpy(&info.status, &network, sizeof(network_settings_t));

    ip4addr_ntoa_r((const ip_addr_t *)&IPAddress, info.status.ip, IPADDR_STRLEN_MAX);

    if(info.status.ip_mode == IpMode_DHCP) {
        *info.status.gateway = '\0';
        *info.status.mask = '\0';
    }

    info.is_ethernet = true;
    info.link_up = linkUp;
    info.mbps = 100;
    info.status.services = services;

    struct netif *netif = netif_default; // netif_get_by_index(0);

    if(netif) {

        if(linkUp) {
            ip4addr_ntoa_r(netif_ip_gw4(netif), info.status.gateway, IP4ADDR_STRLEN_MAX);
            ip4addr_ntoa_r(netif_ip_netmask4(netif), info.status.mask, IP4ADDR_STRLEN_MAX);
        }

        sprintf(info.mac, "%02X:%02X:%02X:%02X:%02X:%02X", netif->hwaddr[0], netif->hwaddr[1], netif->hwaddr[2], netif->hwaddr[3], netif->hwaddr[4], netif->hwaddr[5]);
    }

    return &info;
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(newopt) {
        hal.stream.write(",ETH");
#if FTP_ENABLE
        if(services.ftp)
            hal.stream.write(",FTP");
#endif
#if WEBDAV_ENABLE
        if(services.webdav)
            hal.stream.write(",WebDAV");
#endif
#if MDNS_ENABLE
        if(services.mdns)
            hal.stream.write(",mDNS");
#endif
#if SSDP_ENABLE
        if(services.ssdp)
            hal.stream.write(",SSDP");
#endif
    } else {
        hal.stream.write("[IP:");
        hal.stream.write(enet_ip_address());
        hal.stream.write("]\r\n");

        if(active_stream == StreamType_Telnet || active_stream == StreamType_WebSocket) {
            hal.stream.write("[NETCON:");
            hal.stream.write(active_stream == StreamType_Telnet ? "Telnet" : "Websocket");
            hal.stream.write("]" ASCII_EOL);
        }
    }
}

#if MDNS_ENABLE

static void mdns_device_info (struct mdns_service *service, void *txt_userdata)
{
    char build[20] = "build=";

    strcat(build, uitoa(GRBL_BUILD));
    mdns_resp_add_service_txtitem(service, "model=grblHAL", 13);
    mdns_resp_add_service_txtitem(service, (char *)txt_userdata, strlen((char *)txt_userdata));
    mdns_resp_add_service_txtitem(service, build, strlen(build));
}

static void mdns_service_info (struct mdns_service *service, void *txt_userdata)
{
    if(txt_userdata)
        mdns_resp_add_service_txtitem(service, (char *)txt_userdata, strlen((char *)txt_userdata));
}

#endif

void setupServices (void *pvArg)
{
#ifdef __LOCATOR_H__
    // Setup the device locator service.

    uint8_t MACAddress[6];

    LocatorInit();
    lwIPLocalMACGet(MACAddress);
    LocatorMACAddrSet(MACAddress);
    LocatorAppTitleSet("Grbl CNC Controller");
#endif

#if TELNET_ENABLE
        if(network.services.telnet && !services.telnet)
            services.telnet =  telnetd_init(network.telnet_port == 0 ? NETWORK_TELNET_PORT : network.telnet_port);
#endif

#if FTP_ENABLE
        if(network.services.ftp && !services.ftp)
            services.ftp = ftpd_init(network.ftp_port == 0 ? NETWORK_FTP_PORT : network.ftp_port);;
#endif

#if HTTP_ENABLE
        if(network.services.http && !services.http)
            services.http = httpd_init(network.http_port == 0 ? NETWORK_HTTP_PORT : network.http_port);
  #if WEBDAV_ENABLE
        if(network.services.webdav && !services.webdav)
            services.webdav = webdav_init();
  #endif
  #if SSDP_ENABLE
        if(network.services.ssdp && !services.ssdp)
            services.ssdp = ssdp_init(network.http_port);
 #endif
#endif

#if WEBSOCKET_ENABLE
        if(network.services.websocket && !services.websocket)
            services.websocket = websocketd_init(network.websocket_port == 0 ? NETWORK_WEBSOCKET_PORT : network.websocket_port);
#endif

#if MDNS_ENABLE
    if(*network.hostname && network.services.mdns && !services.mdns) {

        mdns_resp_init();

        if((services.mdns = mdns_resp_add_netif(netif_default, network.hostname, MDNS_TTL) == ERR_OK)) {

            mdns_resp_add_service(netif_default, network.hostname, "_device-info", DNSSD_PROTO_TCP, 0, MDNS_TTL, mdns_device_info, "version=" GRBL_VERSION);

            if(services.http)
                mdns_resp_add_service(netif_default, network.hostname, "_http", DNSSD_PROTO_TCP, network.http_port, MDNS_TTL, mdns_service_info, "path=/");
            if(services.webdav)
                mdns_resp_add_service(netif_default, network.hostname, "_webdav", DNSSD_PROTO_TCP, network.http_port, MDNS_TTL, mdns_service_info, "path=/");
            if(services.websocket)
                mdns_resp_add_service(netif_default, network.hostname, "_websocket", DNSSD_PROTO_TCP, network.websocket_port, MDNS_TTL, mdns_service_info, NULL);
            if(services.telnet)
                mdns_resp_add_service(netif_default, network.hostname, "_telnet", DNSSD_PROTO_TCP, network.telnet_port, MDNS_TTL, mdns_service_info, NULL);
            if(services.ftp)
                mdns_resp_add_service(netif_default, network.hostname, "_ftp", DNSSD_PROTO_TCP, network.ftp_port, MDNS_TTL, mdns_service_info, "path=/");

//            mdns_resp_announce(netif_default);
        }
    }
#endif
}

void lwIPHostTimerHandler (void)
{
    bool isLinkUp = PREF(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0)) != 0;

    IPAddress = network.ip_mode == IpMode_Static && isLinkUp ? *(uint32_t *)network.ip : lwIPLocalIPAddrGet();

    if(IPAddress == 0xffffffff)
        IPAddress = 0;

    if(isLinkUp != linkUp && IPAddress) {

        if((linkUp = isLinkUp))
            tcpip_callback(setupServices, 0);

#if TELNET_ENABLE
        telnetd_notify_link_status(linkUp);
#endif
    }

#if TELNET_ENABLE
    if(services.telnet)
        telnetd_poll();
#endif
#if FTP_ENABLE
    if(services.ftp)
        ftpd_poll();
#endif
#if WEBSOCKET_ENABLE
    if(services.websocket)
        websocketd_poll();
#endif
}

bool enet_start (void)
{
    uint32_t User0, User1;
    uint8_t MACAddress[6];

    // Get the MAC address from the user registers.
    PREF(FlashUserGet(&User0, &User1));

    if(nvs_address == 0 || (User0 == 0xFFFFFFFF) || (User1 == 0xFFFFFFFF))
        return false;

    // Convert the 24/24 split MAC address from flash into a 32/16 split MAC address needed by lwIP.
    MACAddress[0] = ((User0 >>  0) & 0xFF);
    MACAddress[1] = ((User0 >>  8) & 0xFF);
    MACAddress[2] = ((User0 >> 16) & 0xFF);
    MACAddress[3] = ((User1 >>  0) & 0xFF);
    MACAddress[4] = ((User1 >>  8) & 0xFF);
    MACAddress[5] = ((User1 >> 16) & 0xFF);

    memcpy(&network, &ethernet, sizeof(network_settings_t));

    if(network.telnet_port == 0)
        network.telnet_port = NETWORK_TELNET_PORT;
    if(network.websocket_port == 0)
        network.websocket_port = NETWORK_WEBSOCKET_PORT;
    if(network.http_port == 0)
        network.http_port = NETWORK_HTTP_PORT;
    if(network.ftp_port == 0)
        network.ftp_port = NETWORK_FTP_PORT;

    //
    // Lower the priority of the Ethernet interrupt handler to less than
    // configMAX_SYSCALL_INTERRUPT_PRIORITY.  This is required so that the
    // interrupt handler can safely call the interrupt-safe FreeRTOS functions
    // if any.
    //

    PREF(SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF));
    PREF(IntPrioritySet(INT_EMAC0, 0xC0));

    PREF(GPIOPinConfigure(GPIO_PF0_EN0LED0));
    PREF(GPIOPinConfigure(GPIO_PF4_EN0LED1));

    PREF(GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4));

    //
    // Set the link status based on the LED0 signal (which defaults to link
    // status in the PHY).
    //
    // linkUp = PREF(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3)) != 0;

//    SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);
    // Register ethernet IRQ handler here, this since the driver layer does not do that

#ifdef __MSP432E401Y__
    PREF(EMACIntRegister(EMAC0_BASE, EMAC0_IRQHandler));
    PREF(IntPrioritySet(INT_EMAC0, ETHERNET_INT_PRIORITY));
    PREF(IntPrioritySet(FAULT_SYSTICK, SYSTICK_INT_PRIORITY));
#endif

    if(network.ip_mode == IpMode_DHCP)
        lwIPInit(configCPU_CLOCK_HZ, MACAddress, 0, 0, 0, IPADDR_USE_DHCP);
    else
        lwIPInit(configCPU_CLOCK_HZ, MACAddress, ntohl(*(uint32_t *)network.ip), ntohl(*(uint32_t *)network.mask), ntohl(*(uint32_t *)network.gateway), network.ip_mode == IpMode_Static ? IPADDR_USE_STATIC : IPADDR_USE_AUTOIP);

#if MDNS_ENABLE || SSDP_ENABLE
    netif_default->flags |= NETIF_FLAG_IGMP;
// TODO: set EMACHashFilterSet for UDP multicast address filtering?
#endif

#if LWIP_NETIF_HOSTNAME
    extern struct netif *netif_default;
    netif_set_hostname(netif_default, network.hostname);
#endif

    return true;
}

static void ethernet_settings_load (void);
static void ethernet_settings_restore (void);
static status_code_t ethernet_set_ip (setting_id_t setting, char *value);
static char *ethernet_get_ip (setting_id_t setting);
static status_code_t ethernet_set_services (setting_id_t setting, uint_fast16_t int_value);
static uint32_t ethernet_get_services (setting_id_t id);

static const setting_group_detail_t ethernet_groups [] = {
    { Group_Root, Group_Networking, "Networking" }
};

static const setting_detail_t ethernet_settings[] = {
    { Setting_NetworkServices, Group_Networking, "Network Services", NULL, Format_Bitfield, netservices, NULL, NULL, Setting_NonCoreFn, ethernet_set_services, ethernet_get_services, NULL, true },
    { Setting_Hostname, Group_Networking, "Hostname", NULL, Format_String, "x(64)", NULL, "64", Setting_NonCore, ethernet.hostname, NULL, NULL, true },
    { Setting_IpMode, Group_Networking, "IP Mode", NULL, Format_RadioButtons, "Static,DHCP,AutoIP", NULL, NULL, Setting_NonCore, &ethernet.ip_mode, NULL, NULL, true },
    { Setting_IpAddress, Group_Networking, "IP Address", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, ethernet_set_ip, ethernet_get_ip, NULL, true },
    { Setting_Gateway, Group_Networking, "Gateway", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, ethernet_set_ip, ethernet_get_ip, NULL, true },
    { Setting_NetMask, Group_Networking, "Netmask", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, ethernet_set_ip, ethernet_get_ip, NULL, true },
    { Setting_TelnetPort, Group_Networking, "Telnet port", NULL, Format_Int16, "####0", "1", "65535", Setting_NonCore, &ethernet.telnet_port, NULL, NULL, true },
#if FTP_ENABLE
    { Setting_FtpPort, Group_Networking, "FTP port", NULL, Format_Int16, "####0", "1", "65535", Setting_NonCore, &ethernet.ftp_port, NULL, NULL, true },
#endif
#if HTTP_ENABLE
    { Setting_HttpPort, Group_Networking, "HTTP port", NULL, Format_Int16, "####0", "1", "65535", Setting_NonCore, &ethernet.http_port, NULL, NULL, true },
#endif
    { Setting_WebSocketPort, Group_Networking, "Websocket port", NULL, Format_Int16, "####0", "1", "65535", Setting_NonCore, &ethernet.websocket_port, NULL, NULL, true }
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t ethernet_settings_descr[] = {
    { Setting_NetworkServices, "Network services to enable. Consult driver documentation for availability." },
    { Setting_Hostname, "Network hostname." },
    { Setting_IpMode, "IP Mode." },
    { Setting_IpAddress, "Static IP address." },
    { Setting_Gateway, "Static gateway address." },
    { Setting_NetMask, "Static netmask." },
    { Setting_TelnetPort, "(Raw) Telnet port number listening for incoming connections." },
#if FTP_ENABLE
    { Setting_FtpPort, "FTP port number listening for incoming connections." },
#endif
#if HTTP_ENABLE
    { Setting_HttpPort, "HTTP port number listening for incoming connections." },
#endif
    { Setting_WebSocketPort, "Websocket port number listening for incoming connections."
                             "NOTE: WebUI requires this to be HTTP port number + 1."
    }
};

#endif

static void ethernet_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&ethernet, sizeof(network_settings_t), true);
}

static setting_details_t setting_details = {
    .groups = ethernet_groups,
    .n_groups = sizeof(ethernet_groups) / sizeof(setting_group_detail_t),
    .settings = ethernet_settings,
    .n_settings = sizeof(ethernet_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = ethernet_settings_descr,
    .n_descriptions = sizeof(ethernet_settings_descr) / sizeof(setting_descr_t),
#endif
    .save = ethernet_settings_save,
    .load = ethernet_settings_load,
    .restore = ethernet_settings_restore
};

static status_code_t ethernet_set_ip (setting_id_t setting, char *value)
{
    ip_addr_t addr;

    if(ip4addr_aton(value, &addr) != 1)
        return Status_InvalidStatement;

    status_code_t status = Status_OK;

    switch(setting) {

        case Setting_IpAddress:
            *((ip_addr_t *)ethernet.ip) = addr;
            break;

        case Setting_Gateway:
            *((ip_addr_t *)ethernet.gateway) = addr;
            break;

        case Setting_NetMask:
            *((ip_addr_t *)ethernet.mask) = addr;
            break;

        default:
            status = Status_Unhandled;
            break;
    }

    return status;
}

static char *ethernet_get_ip (setting_id_t setting)
{
    static char ip[IPADDR_STRLEN_MAX];

    switch(setting) {

        case Setting_IpAddress:
            ip4addr_ntoa_r((const ip_addr_t *)&ethernet.ip, ip, IPADDR_STRLEN_MAX);
            break;

        case Setting_Gateway:
            ip4addr_ntoa_r((const ip_addr_t *)&ethernet.gateway, ip, IPADDR_STRLEN_MAX);
            break;

        case Setting_NetMask:
            ip4addr_ntoa_r((const ip_addr_t *)&ethernet.mask, ip, IPADDR_STRLEN_MAX);
            break;

        default:
            *ip = '\0';
            break;
    }

    return ip;
}

static status_code_t ethernet_set_services (setting_id_t setting, uint_fast16_t int_value)
{
    ethernet.services.mask = int_value & allowed_services.mask;

    return Status_OK;
}

static uint32_t ethernet_get_services (setting_id_t id)
{
    return (uint32_t)ethernet.services.mask & allowed_services.mask;
}

static void ethernet_settings_restore (void)
{
    strcpy(ethernet.hostname, NETWORK_HOSTNAME);

    ip_addr_t addr;

    ethernet.ip_mode = (ip_mode_t)NETWORK_IPMODE;

    if(ip4addr_aton(NETWORK_IP, &addr) == 1)
        *((ip_addr_t *)ethernet.ip) = addr;

    if(ip4addr_aton(NETWORK_GATEWAY, &addr) == 1)
        *((ip_addr_t *)ethernet.gateway) = addr;

#if NETWORK_IPMODE == 0
    if(ip4addr_aton(NETWORK_MASK, &addr) == 1)
        *((ip_addr_t *)ethernet.mask) = addr;
#else
    if(ip4addr_aton("255.255.255.0", &addr) == 1)
        *((ip_addr_t *)ethernet.mask) = addr;
#endif

    ethernet.services.mask = 0;
    ethernet.ftp_port = NETWORK_FTP_PORT;
    ethernet.telnet_port = NETWORK_TELNET_PORT;
    ethernet.http_port = NETWORK_HTTP_PORT;
    ethernet.websocket_port = NETWORK_WEBSOCKET_PORT;
    ethernet.services.mask = allowed_services.mask;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&ethernet, sizeof(network_settings_t), true);
}

static void ethernet_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&ethernet, nvs_address, sizeof(network_settings_t), true) != NVS_TransferResult_OK)
        ethernet_settings_restore();

    ethernet.services.mask &= allowed_services.mask;
}

static void stream_changed (stream_type_t type)
{
    if(type != StreamType_SDCard)
        active_stream = type;

    if(on_stream_changed)
        on_stream_changed(type);
}

bool enet_init (void)
{
    if((nvs_address = nvs_alloc(sizeof(network_settings_t)))) {

        hal.driver_cap.ethernet = On;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_options;

        on_stream_changed = grbl.on_stream_changed;
        grbl.on_stream_changed = stream_changed;

        settings_register(&setting_details);

        allowed_services.mask = networking_get_services_list((char *)netservices).mask;
    }

    return nvs_address != 0;
}

#endif
