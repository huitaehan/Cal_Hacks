idf_component_register(
    SRCS
        "main.c"
        "ruview_shims.c"
        # Vendored unmodified from RuView firmware/esp32-csi-node/main
        "ruview/edge_processing.c"
    INCLUDE_DIRS "." "ruview"
    REQUIRES esp_wifi esp_netif esp_event nvs_flash esp_timer
)
