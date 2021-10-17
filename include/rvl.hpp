#ifndef TOPIC_COMPRESSION_RVL_HPP
#define TOPIC_COMPRESSION_RVL_HPP

// This algorithm is from (MIT)
// Wilson, A. D. (2017, October). Fast lossless depth image compression.
// In Proceedings of the 2017 ACM International Conference on Interactive Surfaces and Spaces (pp. 100-105). ACM.
// Code is from the RVL paper (Wilson, 2017). Modified for use with ROS.

void EncodeVLE(int value, int*& pBuffer, int& word, int& nibblesWritten)
{
    do {
        int nibble = value & 0x7; // lower 3 bits
        if (value >>= 3) nibble |= 0x8; // more to come
        word <<= 4;
        word |= nibble;
        if (++nibblesWritten == 8) { // output word
            *pBuffer++ = word;
            nibblesWritten = 0;
            word = 0;
        }
    } while (value);
}

int DecodeVLE(int*& pBuffer, int& word, int& nibblesWritten)
{
    unsigned int nibble;
    int value = 0, bits = 29;
    do {
        if (!nibblesWritten) {
            word = *pBuffer++; // load word
            nibblesWritten = 8;
        }
        nibble = word & 0xf0000000;
        value |= (nibble << 1) >> bits;
        word <<= 4;
        nibblesWritten--;
        bits -= 3;
    } while (nibble & 0x80000000);
    return value;
}

int CompressRVL(uint16_t* input, char* output, int numPixels)
{
    int* buffer = (int*)output;
    int* pBuffer = (int*)output;
    int word = 0;
    int nibblesWritten = 0;
    uint16_t* end = input + numPixels;
    uint16_t previous = 0;
    while (input != end) {
        int zeros = 0, nonzeros = 0;
        for (; (input != end) && !*input; input++, zeros++);
        EncodeVLE(zeros, pBuffer, word, nibblesWritten); // number of zeros
        for (uint16_t* p = input; (p != end) && *p++; nonzeros++);
        EncodeVLE(nonzeros, pBuffer, word, nibblesWritten); // number of nonzeros
        for (int i = 0; i < nonzeros; i++) {
            uint16_t current = *input++;
            int delta = current - previous;
            int positive = (delta << 1) ^ (delta >> 31);
            EncodeVLE(positive, pBuffer, word, nibblesWritten); // nonzero value
            previous = current;
        }
    }

    if (nibblesWritten) // last few values
        *pBuffer++ = word << 4 * (8 - nibblesWritten);

    return int((char*)pBuffer - (char*)buffer); // num bytes
}

void DecompressRVL(char* input, uint16_t* output, int numPixels)
{
    int* buffer = (int*)input;
    int* pBuffer = (int*)input;
    int word = 0;
    int nibblesWritten = 0;
    uint16_t current, previous = 0;
    int numPixelsToDecode = numPixels;
    while (numPixelsToDecode) {
        int zeros = DecodeVLE(pBuffer, word, nibblesWritten); // number of zeros
        numPixelsToDecode -= zeros;
        for (; zeros; zeros--)
            *output++ = 0;
        int nonzeros = DecodeVLE(pBuffer, word, nibblesWritten); // number of nonzeros
        numPixelsToDecode -= nonzeros;
        for (; nonzeros; nonzeros--) {
            int positive = DecodeVLE(pBuffer, word, nibblesWritten); // nonzero value
            int delta = (positive >> 1) ^ -(positive & 1);
            current = previous + delta;
            *output++ = current;
            previous = current;
        }
    }
}
#endif //TOPIC_COMPRESSION_RVL_HPP
