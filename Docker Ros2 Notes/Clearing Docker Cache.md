# Clearing Docker Cache
To clear the Docker cache and free up space, follow these steps:

## Remove Stopped Containers

Open a terminal or command prompt.

Run `docker container prune -f` to remove all stopped containers.

## Remove Unused Images

Run `docker image prune -f` to remove dangling images (not associated with any container).

To remove all unused images, including tagged ones, run `docker image prune -a -f`.

## Remove Unused Volumes

Run `docker volume prune -f` to delete all unused volumes.

## Remove Build Cache

Run `docker builder prune -f` to clear the build cache.

For specific builders, use `docker buildx prune --builder <builder-name> -f`.

## Remove Unused Networks

Run `docker network prune -f` to clean up unused networks.

## Remove All Unused Artifacts

Run `docker system prune -f` to remove all unused containers, images, networks, and build cache.

To include volumes, use `docker system prune --volumes -af`.

These commands will help reclaim disk space while ensuring unused Docker artifacts are cleared.

## Learn more:
1. https://depot.dev/blog/docker-clear-cache
2. https://forums.docker.com/t/how-to-delete-cache/5753
3. https://www.freecodecamp.org/news/docker-cache-tutorial/
