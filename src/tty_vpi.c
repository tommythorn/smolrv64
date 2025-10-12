#include <vpi_user.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>

// Global variables for TTY state
static int tty_initialized = 0;
static struct termios orig_termios;

// Initialize TTY for non-blocking, non-canonical mode
static int init_tty(void) {
    struct termios new_termios;
    
    if (tty_initialized) return 0;
    
    // Get current terminal attributes
    if (tcgetattr(STDIN_FILENO, &orig_termios) < 0) {
        vpi_printf("Error getting terminal attributes: %s\n", strerror(errno));
        return -1;
    }
    
    // Set up new terminal attributes
    new_termios = orig_termios;
    new_termios.c_lflag &= ~(ICANON | ECHO); // Disable canonical mode and echo
    new_termios.c_cc[VMIN] = 0;  // Non-blocking read
    new_termios.c_cc[VTIME] = 0; // No timeout
    
    if (tcsetattr(STDIN_FILENO, TCSANOW, &new_termios) < 0) {
        vpi_printf("Error setting terminal attributes: %s\n", strerror(errno));
        return -1;
    }
    
    // Set stdin to non-blocking
    int flags = fcntl(STDIN_FILENO, F_GETFL);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    
    tty_initialized = 1;
    vpi_printf("TTY initialized for non-blocking input\n");
    return 0;
}

// Restore original TTY settings
static void restore_tty(void) {
    if (tty_initialized) {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
        int flags = fcntl(STDIN_FILENO, F_GETFL);
        fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
        tty_initialized = 0;
        vpi_printf("\nTTY restored\n");
    }
}

// VPI function to read from TTY
static PLI_INT32 tty_read_compiled(PLI_BYTE8 *_unused) {
    (void) _unused;
    s_vpi_value arg_value;
    char ch;
    int result;
    
    // Initialize TTY if needed
    if (!tty_initialized) {
        if (init_tty() < 0) {
            arg_value.format = vpiIntVal;
            arg_value.value.integer = -1;
            vpi_put_value(vpi_handle(vpiSysTfCall, NULL), &arg_value, NULL, vpiNoDelay);
            return 0;
        }
    }
    
    // Try to read a character
    result = read(STDIN_FILENO, &ch, 1);
    
    if (result > 0) {
        // Character available
        arg_value.format = vpiIntVal;
        arg_value.value.integer = (int)ch;
    } else if (result == 0 || (result < 0 && errno == EAGAIN)) {
        // No data available
        arg_value.format = vpiIntVal;
        arg_value.value.integer = -1;
    } else {
        // Error
        arg_value.format = vpiIntVal;
        arg_value.value.integer = -2;
    }
    
    // Return the value to Verilog
    vpi_put_value(vpi_handle(vpiSysTfCall, NULL), &arg_value, NULL, vpiNoDelay);
    return 0;
}

// VPI function to write to TTY
static PLI_INT32 tty_write_compiled(PLI_BYTE8 *_unused) {
    (void) _unused;
    vpiHandle systf_handle, arg_iterator, arg_handle;
    s_vpi_value arg_value;
    
    systf_handle = vpi_handle(vpiSysTfCall, NULL);
    arg_iterator = vpi_iterate(vpiArgument, systf_handle);
    
    if (arg_iterator) {
        arg_handle = vpi_scan(arg_iterator);
        if (arg_handle) {
            arg_value.format = vpiIntVal;
            vpi_get_value(arg_handle, &arg_value);
            
            char ch = (char)arg_value.value.integer;
            while (write(STDOUT_FILENO, &ch, 1) == 0)
              ;
            fflush(stdout);
        }
        vpi_free_object(arg_iterator);
    }
    
    return 0;
}

// VPI function to write string to TTY
static PLI_INT32 tty_write_string_compiled(PLI_BYTE8 *_unused) {
    (void) _unused;
    vpiHandle systf_handle, arg_iterator, arg_handle;
    s_vpi_value arg_value;
    
    systf_handle = vpi_handle(vpiSysTfCall, NULL);
    arg_iterator = vpi_iterate(vpiArgument, systf_handle);
    
    if (arg_iterator) {
        arg_handle = vpi_scan(arg_iterator);
        if (arg_handle) {
            arg_value.format = vpiStringVal;
            vpi_get_value(arg_handle, &arg_value);
            
            printf("%s", arg_value.value.str);
            fflush(stdout);
        }
        vpi_free_object(arg_iterator);
    }
    
    return 0;
}

// Cleanup function called when simulation ends
static PLI_INT32 tty_cleanup(struct t_cb_data *_unused) {
    (void) _unused;
    restore_tty();
    return 0;
}

void register_tty_vpi(void) {
    s_vpi_systf_data tf_data;
    
    // Register tty_read function
    tf_data.type = vpiSysFunc;
    tf_data.sysfunctype = vpiIntFunc;
    tf_data.tfname = "$tty_read";
    tf_data.calltf = tty_read_compiled;
    tf_data.compiletf = NULL;
    tf_data.sizetf = NULL;
    tf_data.user_data = NULL;
    vpi_register_systf(&tf_data);
    
    // Register tty_write function
    tf_data.type = vpiSysTask;
    tf_data.tfname = "$tty_write";
    tf_data.calltf = tty_write_compiled;
    tf_data.compiletf = NULL;
    tf_data.sizetf = NULL;
    tf_data.user_data = NULL;
    vpi_register_systf(&tf_data);
    
    // Register tty_write_string function
    tf_data.type = vpiSysTask;
    tf_data.tfname = "$tty_write_string";
    tf_data.calltf = tty_write_string_compiled;
    tf_data.compiletf = NULL;
    tf_data.sizetf = NULL;
    tf_data.user_data = NULL;
    vpi_register_systf(&tf_data);
    
    // Register cleanup callback
    s_cb_data cb_data;
    cb_data.reason = cbEndOfSimulation;
    cb_data.cb_rtn = tty_cleanup;
    cb_data.obj = NULL;
    cb_data.time = NULL;
    cb_data.value = NULL;
    cb_data.user_data = NULL;
    vpi_register_cb(&cb_data);
}

// Register VPI functions
void (*vlog_startup_routines[])(void) = {
    register_tty_vpi,
    0
};
