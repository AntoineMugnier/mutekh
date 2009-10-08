
#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <elfpp/object>
#include <elfpp/segment>
#include <dpp/foreach>

int main(int argc, char **argv)
{
	if ( argc < 2 )
		return 1;

	elfpp::object bin(argv[1]);
	int fd = open(argv[1], O_RDONLY);

	FOREACH( _seg, bin.get_segment_table() ) {
		const elfpp::segment &seg = *_seg;
		void *addr = (void*)seg.get_paddr();
		size_t size = seg.get_mem_size();
		uintptr_t offset = seg.get_file_offset();
		int flags = 0;

		if ( seg.get_flags() & elfpp::PF_X )
			flags |= PROT_EXEC;
		if ( seg.get_flags() & elfpp::PF_R )
			flags |= PROT_READ;
		if ( seg.get_flags() & elfpp::PF_W )
			flags |= PROT_WRITE;

		void* ret = mmap((void*)addr, size, 
						 PROT_WRITE,
						 MAP_FIXED|MAP_PRIVATE|MAP_ANON,
						 0, 0);
		if ( ret == MAP_FAILED ) {
			perror("mmap");
			return 1;
		}
		assert( ret == (void*)addr );
		memset(addr, 0, size);
		lseek(fd, offset, SEEK_SET);
		read(fd, addr, seg.get_file_size());
		mprotect(addr, size, flags);
	}

	typedef void (entry_t)();
	entry_t *func;
	func = (entry_t*)bin.get_entry_point();
	func();
	return 0;
}
