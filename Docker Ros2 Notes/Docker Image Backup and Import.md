# Docker Image Backup and Import

Backing up and importing Docker images ensures portability and recovery of your containerized applications. Below are the steps to achieve this effectively.

## Backup a Docker Image

Identify the Image List all available images to find the one you want to back up:
```
docker images
```
Save the Image Use the docker save command to export the image as a .tar file:
```
docker save -o backup-image.tar <image_name>
```
Verify Backup Ensure the .tar file is created in the specified location:
```
ls -lh backup-image.tar
```
## Import a Docker Image

Transfer the Backup Move the .tar file to the target system where you want to import it.

Load the Image Use the docker load command to import the image from the .tar file:
```
docker load -i backup-image.tar
```
Verify Import Check if the image is successfully loaded:
```
docker image ls
```
## Best Practices

Compression: Compress large backups using gzip for efficient storage:
```
docker save <image_name> | gzip > backup-image.tar.gz
```
Tagging: Use meaningful tags for images before saving them:
```
docker tag <image_id> <repository>:<tag>
```
Storage: Store backups in secure and redundant locations, such as cloud storage or external drives.

These steps ensure seamless portability and recovery of Docker images across environments.

## Learn more:
1. https://docs.docker.com/desktop/settings-and-maintenance/backup-and-restore/
2. https://www.geeksforgeeks.org/devops/export-and-import-docker-containers-and-images/
3. https://stackoverflow.com/questions/26331651/how-can-i-backup-a-docker-container-with-its-data-volumes

---

# Unzip .gz Files in Linux

A .gz file is a single file compressed using the gzip algorithm, commonly used in Unix/Linux systems. You can easily decompress it using built-in commands without installing extra tools.

## Using gunzip

Step 1: Open the terminal and navigate to the directory containing your .gz file:
```
cd /path/to/file
```
Step 2: Run the gunzip command:
```
gunzip filename.gz
```
- This will extract the file and remove the .gz version.
- 
- To keep the original compressed file, use:
```
gunzip -k filename.gz
```
## Using gzip -d

**Step 1**: Navigate to the file location. **Step 2**: Run:
```
gzip -d filename.gz
```
- This works exactly like gunzip.
- 
- To keep the .gz file:
```
gzip -dk filename.gz
```
## Extracting .tar.gz Archives

If your file is a .tar.gz (a tar archive compressed with gzip), use:
```
tar -xzf archive.tar.gz
```
- -x → extract
- -z → decompress gzip
- -f → specify filename

## Verification

After extraction, list files to confirm:
```
ls -l
```
You should see the decompressed file without the .gz extension.

## Tips

- Use GUI extraction by right-clicking the .gz file and selecting Extract Here if you prefer not to use the terminal.
- For large files, ensure you have enough disk space before extraction.
- These methods work on all major Linux distributions and macOS, making .gz handling quick and efficient.