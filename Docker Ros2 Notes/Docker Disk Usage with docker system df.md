# Docker Disk Usage with docker system df

---

The docker system df command displays how much disk space Docker components (images, containers, volumes, build cache) are using, along with reclaimable space. It’s useful for monitoring and cleaning up unused resources.

## Example:
```
# Summary of Docker disk usage
docker system df

# Detailed view with per-item breakdown
docker system df -v

# JSON formatted output for scripting
docker system df --format json
```

**Basic Summary View** Running docker system df without options shows a table with **TYPE**, **TOTAL**, **ACTIVE**, **SIZE**, and **RECLAIMABLE** for each category. Example output:

```
TYPE TOTAL ACTIVE SIZE RECLAIMABLE
Images 5 2 1.24GB 800MB (64%)
Containers 3 1 50MB 20MB (40%)
Local Volumes 4 2 300MB 100MB (33%)
Build Cache 6 0 500MB 500MB (100%)
```

- **TOTAL**: Number of items in that category.
- **ACTIVE**: Items currently in use.
- **SIZE**: Total disk space used.
- **RECLAIMABLE**: Space that can be freed by removing unused items.

**Verbose Mode for Details** Use -v or --verbose to list each image, container, and volume with its individual size and shared/unique usage. Example:
```
docker system df -v
```
This helps identify large unused images or volumes consuming space.

**JSON Output for Automation** For integration with scripts or monitoring tools, format the output as JSON:
```
docker system df --format json | jq
```
This returns structured data for programmatic parsing.

## Key Considerations

- Network objects are not shown because they don’t consume disk space.
- Reclaimable space can be freed using cleanup commands like:
```
docker image prune
docker container prune
docker volume prune
docker builder prune
```
- Regular checks help prevent running out of disk space on Docker hosts.

By combining `docker system df -v` for investigation and prune commands for cleanup, you can efficiently manage Docker’s disk footprint.

Learn more:
### docker system df | Docker Docs
https://docs.docker.com/reference/cli/docker/system/df/

### How to analyze disk usage of a Docker container - Stack Overflow
https://stackoverflow.com/questions/26753087/how-to-analyze-disk-usage-of-a-docker-container

### How to use docker system df command to check disk usage | LabEx
https://labex.io/tutorials/docker-how-to-use-docker-system-df-command-to-check-disk-usage-555247