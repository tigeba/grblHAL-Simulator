FROM debian:bookworm-slim AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build
COPY . .

# Initialize grbl submodule if not present
RUN if [ ! -f src/grbl/CMakeLists.txt ]; then \
        git init && git submodule update --init src/grbl; \
    fi

RUN rm -rf build && mkdir -p build && cd build && cmake .. && make -j$(nproc)

# --- Runtime ---
FROM debian:bookworm-slim

COPY --from=builder /build/build/grblHAL_sim /usr/local/bin/grblHAL_sim

# Default port 23 (telnet/grbl network) inside container
EXPOSE 23

ENTRYPOINT ["grblHAL_sim"]
CMD ["-p", "23"]
