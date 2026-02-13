// Credit to Daniil Zykov for this btw

#include <WiFi.h>
#include <ESPmDNS.h>
#include "lwip/etharp.h"
#include "lwip/netif.h"

const char* ssid = "NinerWiFi-Guest";
const char* password = "";

// Forward declarations
void scanNetwork();
void checkARPCache();
String getMACFromARP(IPAddress ip);
String identifyVendor(String mac);
bool pingIP(IPAddress ip);
String getHostname(IPAddress ip);
void browseMDNS();

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\nESP32 Network Device Scanner");
  Serial.println("=============================\n");
  
  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.print("Connecting to WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nFailed to connect!");
    return;
  }
  
  Serial.println("\nConnected!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Gateway (Router): ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("Subnet Mask: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.println();
  
  // Start mDNS
  if (MDNS.begin("esp32scanner")) {
    Serial.println("mDNS responder started\n");
  }
  
  Serial.println("=== Scanning Network ===\n");
  scanNetwork();
}

void scanNetwork() {
  IPAddress myIP = WiFi.localIP();
  IPAddress gateway = WiFi.gatewayIP();
  
  Serial.println("Method 1: ARP Cache Check");
  Serial.println("-------------------------");
  checkARPCache();
  
  Serial.println("\nMethod 2: Active IP Scan with ARP");
  Serial.println("----------------------------------");
  
  int devicesFound = 0;
  
  // Scan the subnet (typically 192.168.x.0/24)
  for (int i = 1; i < 255; i++) {
    IPAddress testIP(myIP[0], myIP[1], myIP[2], i);
    
    // Skip our own IP
    if (testIP == myIP) {
      Serial.print("✓ ");
      Serial.print(testIP);
      Serial.print(" - THIS ESP32 (");
      Serial.print(WiFi.macAddress());
      Serial.println(")");
      devicesFound++;
      continue;
    }
    
    // Identify gateway
    if (testIP == gateway) {
      Serial.print("✓ ");
      Serial.print(testIP);
      Serial.print(" - GATEWAY/ROUTER");
      
      // Try to get MAC from ARP
      String mac = getMACFromARP(testIP);
      if (mac.length() > 0) {
        Serial.print(" (MAC: ");
        Serial.print(mac);
        Serial.print(")");
      }
      Serial.println();
      devicesFound++;
      continue;
    }
    
    // Try ping + ARP lookup
    if (pingIP(testIP)) {
      devicesFound++;
      Serial.print("✓ ");
      Serial.print(testIP);
      
      // Get MAC address via ARP
      delay(50); // Give time for ARP entry
      String mac = getMACFromARP(testIP);
      
      if (mac.length() > 0) {
        Serial.print(" - MAC: ");
        Serial.print(mac);
        
        // Identify vendor from MAC
        String vendor = identifyVendor(mac);
        if (vendor.length() > 0) {
          Serial.print(" (");
          Serial.print(vendor);
          Serial.print(")");
        }
      }
      
      // Try mDNS hostname
      String hostname = getHostname(testIP);
      if (hostname.length() > 0) {
        Serial.print(" - Name: ");
        Serial.print(hostname);
      }
      
      Serial.println();
    }
    
    delay(100); // Delay between scans
    
    // Progress indicator every 50 IPs
    if (i % 50 == 0) {
      Serial.print("... scanned ");
      Serial.print(i);
      Serial.println(" addresses");
    }
  }
  
  Serial.println("\n-------------------------");
  Serial.print("Total devices found: ");
  Serial.println(devicesFound);
  
  Serial.println("\n=== mDNS Service Discovery ===\n");
  browseMDNS();
}

void checkARPCache() {
  // Access lwIP ARP table
  struct netif *netif_ptr = netif_default;
  
  if (netif_ptr == NULL) {
    Serial.println("No network interface available");
    return;
  }
  
  Serial.println("Current ARP Cache:");
  
  int entries = 0;
  for (int i = 0; i < ARP_TABLE_SIZE; i++) {
    ip4_addr_t *ipaddr;
    struct netif *netif;
    struct eth_addr *ethaddr;
    
    if (etharp_get_entry(i, &ipaddr, &netif, &ethaddr) == 1) {
      entries++;
      
      Serial.print("  ");
      Serial.print(ip4addr_ntoa(ipaddr));
      Serial.print(" -> ");
      
      char macStr[18];
      sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
              ethaddr->addr[0], ethaddr->addr[1], ethaddr->addr[2],
              ethaddr->addr[3], ethaddr->addr[4], ethaddr->addr[5]);
      Serial.print(macStr);
      
      String vendor = identifyVendor(String(macStr));
      if (vendor.length() > 0) {
        Serial.print(" (");
        Serial.print(vendor);
        Serial.print(")");
      }
      
      Serial.println();
    }
  }
  
  if (entries == 0) {
    Serial.println("  (empty - will populate during scan)");
  }
}

String getMACFromARP(IPAddress ip) {
  struct netif *netif_ptr = netif_default;
  if (netif_ptr == NULL) return "";
  
  ip4_addr_t ipaddr;
  IP4_ADDR(&ipaddr, ip[0], ip[1], ip[2], ip[3]);
  
  for (int i = 0; i < ARP_TABLE_SIZE; i++) {
    ip4_addr_t *cached_ip;
    struct netif *netif;
    struct eth_addr *ethaddr;
    
    if (etharp_get_entry(i, &cached_ip, &netif, &ethaddr) == 1) {
      if (cached_ip && ip4_addr_cmp(cached_ip, &ipaddr)) {
        char macStr[18];
        sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
                ethaddr->addr[0], ethaddr->addr[1], ethaddr->addr[2],
                ethaddr->addr[3], ethaddr->addr[4], ethaddr->addr[5]);
        return String(macStr);
      }
    }
  }
  
  return "";
}

String identifyVendor(String mac) {
  // Extract OUI (first 3 octets) for vendor identification
  String oui = mac.substring(0, 8);
  oui.replace(":", "");
  oui.toUpperCase();
  
  // Common vendor OUIs (first 6 hex digits)
  if (oui.startsWith("00:50:F2") || oui.startsWith("28:6D:CD") || 
      oui.startsWith("3C:22:FB") || oui.startsWith("E8:84:A5")) return "Apple";
  if (oui.startsWith("00:1A:11") || oui.startsWith("34:2E:B7") || 
      oui.startsWith("88:36:6C")) return "Samsung";
  if (oui.startsWith("00:50:56") || oui.startsWith("00:0C:29")) return "VMware";
  if (oui.startsWith("08:00:27")) return "VirtualBox";
  if (oui.startsWith("B8:27:EB") || oui.startsWith("DC:A6:32") || 
      oui.startsWith("E4:5F:01")) return "Raspberry Pi";
  if (oui.startsWith("AC:DE:48") || oui.startsWith("24:0A:C4") || 
      oui.startsWith("30:AE:A4")) return "Espressif (ESP32)";
  if (oui.startsWith("00:E0:4C") || oui.startsWith("D8:47:32")) return "Realtek";
  if (oui.startsWith("00:11:22") || oui.startsWith("00:25:9C")) return "Google";
  if (oui.startsWith("F4:F5:D8") || oui.startsWith("FC:E9:98")) return "Amazon";
  
  return ""; // Unknown vendor
}

bool pingIP(IPAddress ip) {
  WiFiClient client;
  client.setTimeout(200);
  
  // Try common ports
  int ports[] = {80, 443, 8080, 22, 445};
  
  for (int i = 0; i < 5; i++) {
    if (client.connect(ip, ports[i], 200)) {
      client.stop();
      return true;
    }
  }
  
  return false;
}

String getHostname(IPAddress ip) {
  // mDNS hostname resolution (limited functionality)
  return "";
}

void browseMDNS() {
  const char* services[] = {
    "_http",        // Web servers
    "_workstation", // Computers
    "_smb",         // Windows file sharing
    "_afpovertcp",  // Apple file sharing
    "_printer",     // Printers
    "_ipp",         // Internet Printing Protocol
    "_airplay",     // AirPlay devices
    "_googlecast",  // Chromecast
    "_homekit"      // HomeKit devices
  };
  
  for (int i = 0; i < 9; i++) {
    Serial.print("Browsing for ");
    Serial.print(services[i]);
    Serial.println(" services...");
    
    int n = MDNS.queryService(services[i], "tcp");
    
    if (n > 0) {
      for (int j = 0; j < n; j++) {
        Serial.print("  - ");
        Serial.print(MDNS.hostname(j));
        Serial.print(" (");
        Serial.print(MDNS.address(j));
        Serial.print(":");
        Serial.print(MDNS.port(j));
        Serial.println(")");
      }
    }
  }
  
  Serial.println("\nScan complete!");
}

void loop() {
  // Rescan every 60 seconds
  delay(60000);
  Serial.println("\n\n=== Rescanning Network ===\n");
  scanNetwork();
}
