#if defined(MEMORY_DEBUG_INFO)
#define MAX_THREAD_INFO 10

mbed_stats_heap_t heap_info;
mbed_stats_stack_t stack_info[MAX_THREAD_INFO];

int print_debug_info() {
    debug("\nThis message is from debug function");
    debug_if(1, "\nThis message is from debug_if function");
    debug_if(0, "\nSOMETHING WRONG!!! This message from debug_if function shouldn't show on bash");

    printf("\nMemoryStats:");
    mbed_stats_heap_get(&heap_info);
    printf("\n\tBytes allocated currently: %d", heap_info.current_size);
    printf("\n\tMax bytes allocated at a given time: %d", heap_info.max_size);
    printf("\n\tCumulative sum of bytes ever allocated: %d", heap_info.total_size);
    printf("\n\tCurrent number of bytes allocated for the heap: %d", heap_info.reserved_size);
    printf("\n\tCurrent number of allocations: %d", heap_info.alloc_cnt);
    printf("\n\tNumber of failed allocations: %d", heap_info.alloc_fail_cnt);

    mbed_stats_stack_get(&stack_info[0]);
    printf("\nCumulative Stack Info:");
    printf("\n\tMaximum number of bytes used on the stack: %d", stack_info[0].max_size);
    printf("\n\tCurrent number of bytes allocated for the stack: %d", stack_info[0].reserved_size);
    printf("\n\tNumber of stacks stats accumulated in the structure: %d", stack_info[0].stack_cnt);

    mbed_stats_stack_get_each(stack_info, MAX_THREAD_INFO);
    printf("\nThread Stack Info:");
    for (int i = 0; i < MAX_THREAD_INFO; i++) {
        if (stack_info[i].thread_id != 0) {
            printf("\n\tThread: %d", i);
            printf("\n\t\tThread Id: 0x%08X", stack_info[i].thread_id);
            printf("\n\t\tMaximum number of bytes used on the stack: %d", stack_info[i].max_size);
            printf("\n\t\tCurrent number of bytes allocated for the stack: %d", stack_info[i].reserved_size);
            printf("\n\t\tNumber of stacks stats accumulated in the structure: %d", stack_info[i].stack_cnt);
        }
    }

    printf("\nDone...\n\n");
}
#endif /* MEMORY_DEBUG_INFO */
