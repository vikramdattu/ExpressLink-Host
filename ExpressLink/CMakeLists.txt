set(requires    driver esp_timer)
idf_component_register(
    SRC_DIRS        src
    INCLUDE_DIRS    inc
    REQUIRES        ${requires}
)
