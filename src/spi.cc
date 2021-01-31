#include "spi.h"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>

#include "hsg-sim.h"

// Report io-error, return false.
static bool io_problem(const char *msg) {
  perror(msg);
  return false;
}

static void hex_dump(const void *src, size_t length, const char *prefix) {
  const uint8_t *address = (const unsigned char*) src;

  printf("%s | ", prefix);
  if (!src) {
    printf("\e[1;31m(ignored)\e[0m\n");
    return;
  }
  if (length == 0) {
    printf("\e[1;31m(no bytes)\e[0m\n");
    return;
  }
  for (size_t b = 0; b < length; ++b) {
    printf("%02X ", address[b]);
  }
  printf(" | ");
  for (size_t b = 0; b < length; ++b) {
    uint8_t c = address[b];
    printf("%c", (c < 33 || c == 255) ? '.' : c);
  }
  printf("\e[0m\n");
}

SPIHost::~SPIHost() { close(fd_); }

bool SPIHost::Connect(const char *device, const Options &set_options) {
  options_ = set_options;
  if (spi_sim_) return true;

  if (fd_ >= 0)
    close(fd_);  // Was already open.
  if ((fd_ = open(device, O_RDWR)) < 0)
    return io_problem(device);

  options_.mode = 0x55555555;  // See if we get properly set.
  // Right now, we only read the mode, but later should allow to set
  // it via options.
  int ret = ioctl(fd_, SPI_IOC_RD_MODE32, &options_.mode);
  if (ret < 0) return io_problem("Couldn't read mode");
  fprintf(stderr, "SPI mode is: 0x%08x\n", options_.mode);

  ret = ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &options_.bits_per_word);
  if (ret < 0) return io_problem("Setting bits per word");

  // Let's read back to see that we set what we thought.
  ret = ioctl(fd_, SPI_IOC_RD_BITS_PER_WORD, &options_.bits_per_word);
  if (ret < 0) return io_problem("Couldn't read bits per word");
  if (options_.bits_per_word != set_options.bits_per_word)
    return io_problem("Bits per word didn't stick");

  ret = ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &options_.speed_hz);
  if (ret < 0) return io_problem("Setting speed");

  ret = ioctl(fd_, SPI_IOC_RD_MAX_SPEED_HZ, &options_.speed_hz);
  if (ret < 0) return io_problem("Reading speed");
  if (options_.speed_hz != set_options.speed_hz) {
    fprintf(stderr, "Wanted speed: %.3fkHz, actual speed: %.3fkHz\n",
            set_options.speed_hz/1000.0, options_.speed_hz/1000.0);
  }
  return true;
}


bool SPIHost::TransferBuffer(const void *send, void *receive, size_t len,
                             bool is_last_in_transaction) {
  if (spi_sim_) {
    spi_sim_->SendReceive(send, receive, len, is_last_in_transaction);
  } else {
    struct spi_ioc_transfer tr = {};
    tr.tx_buf = (unsigned long)send;
    tr.rx_buf = (unsigned long)receive;
    tr.len = (uint32_t) len;
    tr.speed_hz = options_.speed_hz;
    tr.delay_usecs = 0;
    tr.bits_per_word = options_.bits_per_word;
    tr.cs_change = is_last_in_transaction ? 0 : 1;

    if (options_.mode & SPI_TX_QUAD)
      tr.tx_nbits = 4;
    else if (options_.mode & SPI_TX_DUAL)
      tr.tx_nbits = 2;
    if (options_.mode & SPI_RX_QUAD)
      tr.rx_nbits = 4;
    else if (options_.mode & SPI_RX_DUAL)
      tr.rx_nbits = 2;
    if (!(options_.mode & SPI_LOOP)) {
      if (options_.mode & (SPI_TX_QUAD | SPI_TX_DUAL))
        tr.rx_buf = 0;
      else if (options_.mode & (SPI_RX_QUAD | SPI_RX_DUAL))
        tr.tx_buf = 0;
    }

    if (ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) < 0)
      return io_problem("Sending SPI message");
  }
  if (options_.verbose) {
    hex_dump(send, len, "\t\e[1;36mTX");
    hex_dump(receive, len, "\t\e[1;33mRX");
  }
  return true;
}
