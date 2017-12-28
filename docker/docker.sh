echo ========= Build docker image
docker build -t otus.lessons.06.02 .
echo ========= Check allocator version
docker run --rm -i otus.lessons.06.02 allocator -v
echo ========= Allocate, result will go to ../bin/out.txt
docker run --rm -i otus.lessons.06.02 otus.lessons.06.02
echo ========= Remove docker image
docker rmi otus.lessons.06.01