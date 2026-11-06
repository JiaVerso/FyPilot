/* Check buffer contents/accounting; the host mutex shim does not test scheduling. */
#include "buffer.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
int main(void)
{
    ring_buffer_t rb = {0};
    char storage[5], out[16] = {0};
    assert(ring_buffer_init(&rb, storage, 5) == RT_EOK);
    assert(ring_buffer_init(&rb, storage, 5) == -RT_EBUSY);
    assert(ring_buffer_read(&rb, out, sizeof(out)) == 0);
    assert(ring_buffer_write(&rb, "abcdefg", 7) == 5);
    assert(rb.dropped_count == 2);
    assert(ring_buffer_read(&rb, out, 16) == 5);
    assert(memcmp(out, "cdefg", 5) == 0);
    assert(ring_buffer_read(&rb, out, 1) == 0);
    assert(ring_buffer_write(&rb, "abc", 3) == 3);
    assert(ring_buffer_read(&rb, out, 2) == 2 && memcmp(out, "ab", 2) == 0);
    assert(ring_buffer_write(&rb, "defgh", 5) == 5);
    assert(rb.dropped_count == 3);
    assert(ring_buffer_read(&rb, out, 16) == 5 && memcmp(out, "defgh", 5) == 0);
    for (int i = 0; i < 100; ++i)
    {
        assert(ring_buffer_write(&rb, "abc", 3) == 3);
        assert(ring_buffer_read(&rb, out, 16) == 3 && memcmp(out, "abc", 3) == 0);
    }
    assert(ring_buffer_reset(&rb) == RT_EOK && rb.dropped_count == 0);
    assert(ring_buffer_write(NULL, "a", 1) == 0);
    assert(ring_buffer_write(&rb, NULL, 1) == 0);
    assert(ring_buffer_release_mutex(&rb) == RT_EOK);
    assert(ring_buffer_read(&rb, out, 1) == 0);
    assert(ring_buffer_write(&rb, "a", 1) == 0);
    puts("PASS: buffer overwrite/newest bytes, wrap, empty/overread, reset, invalid lifecycle");
    return 0;
}
