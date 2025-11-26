#include "drivers/ir_barcode_scanner.h"

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "config.h"

// ============================================================================
// INTERNAL CONFIGURATION
// ============================================================================

#define BARCODE_ADC_GPIO              IR_BARCODE_ADC_GPIO
#define BARCODE_ADC_CHANNEL           IR_BARCODE_ADC_CHANNEL

// Minimum duration for a valid bar or space (ms)
#define BARCODE_MIN_SEGMENT_MS        3

// Segments shorter than this are merged into neighbours (ms)
#define BARCODE_MERGE_THRESHOLD_MS    15

// Cooldown time after a decode (ms)
#define BARCODE_COOLDOWN_MS           2000

// Max segments in a single Code 39 character
#define BARCODE_MAX_SEGMENTS          9

// Timeout while recording one block (ms)
#define BARCODE_BLOCK_TIMEOUT_MS      2000

// ============================================================================
// INTERNAL TYPES
// ============================================================================

typedef enum {
    BARCODE_WAIT_WHITE = 0,
    BARCODE_RECORD_SEGMENTS,
    BARCODE_COOLDOWN
} barcode_state_t;

typedef struct {
    bool     is_black;
    uint32_t duration_ms;
} barcode_segment_t;

typedef struct {
    barcode_state_t   state;
    barcode_segment_t segments[BARCODE_MAX_SEGMENTS];
    int               segment_count;
    uint32_t          segment_start_ms;
    bool              last_color;
    uint32_t          cooldown_start_ms;
    char              final_char;
    bool              has_valid_char;
} barcode_scanner_t;

// ============================================================================
// INTERNAL STATE
// ============================================================================

static barcode_scanner_t g_scanner = {0};

// ============================================================================
// INTERNAL HELPERS
// ============================================================================

// Get current time in milliseconds
static inline uint32_t barcode_now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

// Read raw ADC value from barcode channel
static uint16_t barcode_read_adc_raw_internal(void) {
    adc_select_input(BARCODE_ADC_CHANNEL);
    sleep_us(5);
    return adc_read();
}

// Convert ADC value to logical black / white
static bool barcode_read_ir(void) {
    uint16_t adc_value = barcode_read_adc_raw_internal();

    bool is_black;

#if IR_BARCODE_WHITE_HIGH
    // White is HIGH, black is LOW
    is_black = (adc_value < IR_BARCODE_THRESHOLD);
#else
    // White is LOW, black is HIGH
    is_black = (adc_value > IR_BARCODE_THRESHOLD);
#endif

    return is_black;
}

// Reset scanner to idle state
static void barcode_reset_internal(void) {
    g_scanner.state           = BARCODE_WAIT_WHITE;
    g_scanner.segment_count   = 0;
    g_scanner.segment_start_ms = barcode_now_ms();
    g_scanner.last_color      = false;
}

// Merge very small segments into their neighbours
static void merge_small_segments(barcode_scanner_t *scanner) {
    int i = 0;

    while (i < scanner->segment_count - 1) {
        if (scanner->segments[i].duration_ms < BARCODE_MERGE_THRESHOLD_MS) {
            scanner->segments[i + 1].duration_ms += scanner->segments[i].duration_ms;

            for (int j = i; j < scanner->segment_count - 1; j++) {
                scanner->segments[j] = scanner->segments[j + 1];
            }

            scanner->segment_count--;
        } else {
            i++;
        }
    }
}

// Validate colour pattern for Code 39
static bool validate_color_pattern(barcode_segment_t *segments, int count) {
    if (count != BARCODE_MAX_SEGMENTS) {
        return false;
    }

    if (!segments[0].is_black) {
        return false;
    }

    for (int i = 0; i < count; i++) {
        bool expected_black = (i % 2 == 0);
        if (segments[i].is_black != expected_black) {
            return false;
        }
    }

    return true;
}

// Code 39 pattern lookup table
static const struct {
    const char *pattern;
    char        character;
} CODE39_TABLE[] = {
    {"NWNNWNWNN", '*'},
    {"WNNNNWNNW", 'A'},
    {"NNWNNWNNW", 'B'},
    {"WNWNNWNNN", 'C'},
    {"NNNNWWNNW", 'D'},
    {"WNNNWWNNN", 'E'},
    {"NNWNWWNNN", 'F'},
    {"NNNNNWWNW", 'G'},
    {"WNNNNWWNN", 'H'},
    {"NNWNNWWNN", 'I'},
    {"NNNNWWWNN", 'J'},
    {"WNNNNNNWW", 'K'},
    {"NNWNNNNWW", 'L'},
    {"WNWNNNNWN", 'M'},
    {"NNNNWNNWW", 'N'},
    {"WNNNWNNWN", 'O'},
    {"NNWNWNNWN", 'P'},
    {"NNNNNNWWW", 'Q'},
    {"WNNNNNWWN", 'R'},
    {"NNWNNNWWN", 'S'},
    {"NNNNWNWWN", 'T'},
    {"WWNNNNNNW", 'U'},
    {"NWWNNNNNW", 'V'},
    {"WWWNNNNNN", 'W'},
    {"NWNWNNNNW", 'X'},
    {"WWNWNNNNN", 'Y'},
    {"NWWNNNNNN", 'Z'},
    {"NNNWWNWNN", '0'},
    {"WNNWNNNNW", '1'},
    {"NNWWNNNNW", '2'},
    {"WNWWNNNNN", '3'},
    {"NNNWWNNNW", '4'},
    {"WNNWWNNNN", '5'},
    {"NNWWWNNNN", '6'},
    {"NNNWWNNNW", '7'},
    {"WNNNWNNNW", '8'},
    {"NNWWWNNNW", '9'},
};

// Classify segments as narrow / wide by picking 3 longest
static bool classify_segments(barcode_segment_t *segments, int count, char *pattern_out) {
    if (count != BARCODE_MAX_SEGMENTS) {
        return false;
    }

    typedef struct {
        uint32_t duration;
        int      index;
    } segment_duration_t;

    segment_duration_t tmp[BARCODE_MAX_SEGMENTS];

    for (int i = 0; i < count; i++) {
        tmp[i].duration = segments[i].duration_ms;
        tmp[i].index    = i;
    }

    for (int i = 0; i < count - 1; i++) {
        for (int j = i + 1; j < count; j++) {
            if (tmp[j].duration > tmp[i].duration) {
                segment_duration_t t = tmp[i];
                tmp[i] = tmp[j];
                tmp[j] = t;
            }
        }
    }

    bool is_wide[BARCODE_MAX_SEGMENTS] = {0};

    for (int k = 0; k < 3; k++) {
        is_wide[tmp[k].index] = true;
    }

    int narrow_count = 0;
    int wide_count   = 0;

    for (int i = 0; i < count; i++) {
        if (is_wide[i]) {
            pattern_out[i] = 'W';
            wide_count++;
        } else {
            pattern_out[i] = 'N';
            narrow_count++;
        }
    }

    pattern_out[count] = '\0';

    if (narrow_count != 6 || wide_count != 3) {
        return false;
    }

    return true;
}

// Lookup Code39 character for a forward pattern
static char lookup_code39_character(const char *pattern) {
    int table_size = (int)(sizeof(CODE39_TABLE) / sizeof(CODE39_TABLE[0]));

    for (int i = 0; i < table_size; i++) {
        if (strcmp(pattern, CODE39_TABLE[i].pattern) == 0) {
            return CODE39_TABLE[i].character;
        }
    }

    return '\0';
}

// Lookup Code39 character, trying both forward and reversed pattern
static char lookup_code39_character_bidirectional(const char *pattern) {
    char ch = lookup_code39_character(pattern);
    if (ch != '\0') {
        return ch;
    }

    char reversed[BARCODE_MAX_SEGMENTS + 1];
    int  length = (int)strlen(pattern);

    for (int i = 0; i < length; i++) {
        reversed[i] = pattern[length - 1 - i];
    }
    reversed[length] = '\0';

    return lookup_code39_character(reversed);
}

// Decode 9 segments into a Code39 character
static bool decode_block(barcode_scanner_t *scanner, char *result) {
    if (!validate_color_pattern(scanner->segments, scanner->segment_count)) {
        return false;
    }

    char pattern[BARCODE_MAX_SEGMENTS + 1];

    if (!classify_segments(scanner->segments, scanner->segment_count, pattern)) {
        return false;
    }

    char ch = lookup_code39_character_bidirectional(pattern);
    if (ch == '\0') {
        return false;
    }

    *result = ch;
    return true;
}

// ============================================================================
// PUBLIC IMPLEMENTATION
// ============================================================================

// Initialise barcode scanner
void ir_barcode_scanner_init(void) {
    adc_gpio_init(BARCODE_ADC_GPIO);
    memset(&g_scanner, 0, sizeof(g_scanner));
    g_scanner.state = BARCODE_WAIT_WHITE;
    g_scanner.segment_start_ms = barcode_now_ms();
}

// Update barcode scanner state machine
void ir_barcode_update(void) {
    uint32_t now          = barcode_now_ms();
    bool     current_black = barcode_read_ir();

    switch (g_scanner.state) {
        case BARCODE_WAIT_WHITE: {
            static bool init_done = false;

            if (!init_done) {
                g_scanner.last_color       = current_black;
                g_scanner.segment_start_ms = now;
                init_done                  = true;
                break;
            }

            if (current_black == g_scanner.last_color) {
                break;
            }

            g_scanner.segment_start_ms = now;
            g_scanner.last_color       = current_black;

            if (current_black) {
                g_scanner.segment_count            = 1;
                g_scanner.segments[0].is_black     = true;
                g_scanner.segments[0].duration_ms  = 0;
                g_scanner.state                    = BARCODE_RECORD_SEGMENTS;
            }
            break;
        }

        case BARCODE_RECORD_SEGMENTS: {
            if (current_black != g_scanner.last_color) {
                uint32_t duration = now - g_scanner.segment_start_ms;

                if (duration < BARCODE_MIN_SEGMENT_MS) {
                    g_scanner.segment_start_ms = now;
                    break;
                }

                if (g_scanner.segment_count > 0) {
                    g_scanner.segments[g_scanner.segment_count - 1].duration_ms = duration;
                }

                if (g_scanner.segment_count == BARCODE_MAX_SEGMENTS) {
                    merge_small_segments(&g_scanner);

                    char decoded;
                    if (decode_block(&g_scanner, &decoded)) {
                        g_scanner.final_char     = decoded;
                        g_scanner.has_valid_char = true;
                    }

                    barcode_reset_internal();
                    break;
                }

                if (g_scanner.segment_count < BARCODE_MAX_SEGMENTS) {
                    g_scanner.segments[g_scanner.segment_count].is_black    = current_black;
                    g_scanner.segments[g_scanner.segment_count].duration_ms = 0;
                    g_scanner.segment_count++;
                    g_scanner.segment_start_ms = now;
                    g_scanner.last_color       = current_black;
                } else {
                    barcode_reset_internal();
                }
            } else {
                uint32_t duration = now - g_scanner.segment_start_ms;
                if (duration > BARCODE_BLOCK_TIMEOUT_MS) {
                    barcode_reset_internal();
                }
            }
            break;
        }

        case BARCODE_COOLDOWN: {
            if ((now - g_scanner.cooldown_start_ms) > BARCODE_COOLDOWN_MS) {
                g_scanner.state           = BARCODE_WAIT_WHITE;
                g_scanner.segment_count   = 0;
                g_scanner.segment_start_ms = now;
                g_scanner.last_color      = current_black;
            }
            break;
        }

        default:
            break;
    }
}

// Print last scan data
void ir_barcode_print_data(void) {
    if (!g_scanner.has_valid_char) {
        printf("[BARCODE] No decoded character\n");
        return;
    }

    printf("[BARCODE] Decoded char: '%c'\n", g_scanner.final_char);
}

// Return raw ADC value
uint16_t ir_barcode_read_adc_raw(void) {
    return barcode_read_adc_raw_internal();
}

// Reset scanner and clear state
void ir_barcode_reset(void) {
    barcode_reset_internal();
    g_scanner.final_char     = '\0';
    g_scanner.has_valid_char = false;
}

// Check if a new character is available
bool ir_barcode_has_char(void) {
    return g_scanner.has_valid_char;
}

// Get last decoded character
char ir_barcode_get_char(void) {
    return g_scanner.final_char;
}

// Clear decoded character flag
void ir_barcode_clear_char(void) {
    g_scanner.has_valid_char = false;
    g_scanner.final_char     = '\0';
}
