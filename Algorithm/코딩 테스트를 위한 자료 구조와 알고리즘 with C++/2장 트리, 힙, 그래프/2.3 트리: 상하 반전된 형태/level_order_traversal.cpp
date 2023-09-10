static void levelOrder(node* start)
    {
        if(!start)
            return;
            
        std::queue<node*> q;
        q.push(start);
        
        while(!q.empty())
        {
            int size = q.size();
            
            for(int i = 0; i < size; i++)
            {
                auto current = q.front();
                q.pop();
                
                std::cout << current->position << ", ";
                if(current->first)
                    q.push(current->first);
                if(current->second)
                    q.push(current->second);
            }
            
            std::cout << std::endl;
        }
    }
