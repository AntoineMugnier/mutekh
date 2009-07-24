

#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <list>

#include <dpp/foreach>
#include <dpp/interval_set>

#include <elfpp/object>
#include <elfpp/section>
#include <elfpp/symbol>

struct het_object_s
{
  std::string filename_;
  elfpp::object *obj_;
};

struct het_symbol_s
{
  std::string name_;
  struct het_section_s *section_;
  uint32_t value_;
  size_t size_;
  std::list<elfpp::symbol*> symbols_;
};

typedef std::map<std::string, het_symbol_s *> het_symbols_map_t;

struct het_section_s
{
  std::string name_;
  het_symbols_map_t syms_;
  unsigned int ref_count_;
};

typedef std::map<std::string, het_section_s *> het_sections_map_t;

void error(const char *err)
{
  fprintf(stderr, "%s\n", err);
  exit(1);
}

int main(int argc, char **argv)
{
  std::vector<het_object_s> het_objects(argc - 1);

  // Load input files

  for (int i = 0; i < argc - 1; i++)
    {
      het_object_s &ho = het_objects[i];
      ho.filename_ = argv[i + 1];
      ho.obj_ = new elfpp::object(ho.filename_);
      ho.obj_->parse_symbol_table();
      ho.obj_->load_symbol_data();
    }

  // Process files

  het_sections_map_t het_sections;

  FOREACH(o, het_objects)
    {
      std::cout << " FILE " << o->filename_ << std::endl;

      FOREACH(S, o->obj_->get_section_table())
	{
	  if (!(S->get_flags() & elfpp::SHF_ALLOC))
	    continue;

	  //	  S->set_size(0);

	  std::cout << " SECTION " << S->get_name() << std::endl;

	  // find or create new het-section
	  het_sections_map_t::iterator i = het_sections.find(S->get_name());
	  het_section_s *hsec;

	  if (i == het_sections.end())
	    {
	      hsec = new het_section_s;
	      hsec->name_ = S->get_name();
	      hsec->ref_count_ = 1;
	      het_sections.insert(het_sections_map_t::value_type(hsec->name_, hsec));
	    }
	  else
	    {
	      hsec = i->second;
	      hsec->ref_count_++;	      
	    }

	  // find all section area covered by a symbol
	  typedef dpp::interval_set<uint32_t> is_t;
	  is_t is;

	  is |= is_t::interval_type(S->get_size(), (uint32_t)-1);

	  FOREACH(s, S->get_symbol_table())
	    {
	      uint32_t val = s->second->get_value();
	      size_t size = s->second->get_size();

	      if (size)
		is |= is_t::interval_type(val, val + size);
	    }

	  is = ~is;

	  // create symbols for all orphan section areas in this object
	  FOREACH(i, is)
	    {
	      static int nosym_id = 0;
	      char name[32];
	      sprintf(name, "nosym_%i", nosym_id++);
	      std::cout << std::hex << "in section " << S->get_name() << " 0x" << i->low_bound() << " to 0x" << i->high_bound() << std::endl;
	      elfpp::symbol *sym = new elfpp::symbol(name);

	      sym->set_value(i->low_bound());
	      sym->set_size(i->high_bound() - i->low_bound());
	      sym->set_section(*S);
	      S->add_symbol(*sym);

	      if (S->get_type() != elfpp::SHT_NOBITS)
		sym->set_content(S->get_content() + sym->get_value());
	    }

	  if (S->get_type() != elfpp::SHT_NOBITS)
	      memset(S->get_content(), 0xaa, S->get_size());

	  // find or create new het-symbols
	  FOREACH(s, S->get_symbol_table())
	    {
	      het_symbols_map_t::iterator i = hsec->syms_.find(s->second->get_name());

	      if (i == hsec->syms_.end())
		{
		  // symbol name yet unknown
		  std::cout << " NEW " << s->second->get_name() << std::endl;
		  het_symbol_s *hsym = new het_symbol_s;
		  hsym->name_ = s->second->get_name();
		  hsym->section_ = hsec;
		  hsym->value_ = s->second->get_value();
		  hsym->size_ = s->second->get_size();
		  hsym->symbols_.push_back(s->second);
		  hsec->syms_.insert(het_symbols_map_t::value_type(hsym->name_, hsym));
		}
	      else
		{
		  // already existing symbol name
		  std::cout << " SAME " << s->second->get_name() << std::endl;
		  if (i->second->size_ < s->second->get_size())
		    i->second->size_ = s->second->get_size();
		  i->second->symbols_.push_back(s->second);
		}
	    }
	}
    }

  FILE *conf = fopen("hetlink.conf", "r");
  char buff[256];

  if (!conf)
    error("unable to open configuration file");

  while (char *line = fgets(buff, 256, conf))
    {
      line += strspn(line, " \n\t");
      const char *name = strsep(&line, ", \n\t");
      const char *action = strsep(&line, ", \n\t");

      het_section_s *HS = het_sections[name];

      if (!HS)
	error("unable to find section\n");

      switch (*action)
	{
	case 'c':
	  uint32_t v = 0;
	  FOREACH(Hs, HS->syms_)
	    {
	      FOREACH(s, Hs->second->symbols_)
		(*s)->set_value(v);

	      v += Hs->second->size_;
	    }
	};
    }

#if 0
  FOREACH(HS, het_sections)
    {
      if (HS->second->ref_count_ < 2)
	continue;

      uint32_t v = 0;

      FOREACH(Hs, HS->second->syms_)
	{
	  FOREACH(s, Hs->second->symbols_)
	    (*s)->set_value(v);

	  v += Hs->second->size_;
	}
    }
#endif
  // Write output files

  FOREACH(ho, het_objects)
    {
      ho->obj_->write(ho->filename_ + std::string(".het.o"));
    }

}

